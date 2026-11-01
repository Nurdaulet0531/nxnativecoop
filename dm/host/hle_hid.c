// nn::hid (controller): on the real device shared memory is read through the hid service; the host takes the current state written by the front end (Mac keyboard / controller, iOS virtual gamepad / controller).
// Pro Controller multi-player implementation: each controller is mapped independently to Switch player slots No1..No8 (ids 0..7).
// Controller index 0 -> NpadIdType::No1, index 1 -> NpadIdType::No2, index 2 -> NpadIdType::No3, etc.
// Six-axis sensor and haptics are routed per connected player controller.
#include "hle.h"
#include "hle_haptics.h"
#include "dm_input.h"
#include "front.h"
#include <stdatomic.h>
#include <unistd.h>
#include <strings.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include "motion_fusion.h"

enum { NPAD_NO1 = 0, NPAD_HANDHELD = 0x20 };
enum { STYLE_FULLKEY = 1u << 0, STYLE_HANDHELD = 1u << 1, STYLE_JOYDUAL = 1u << 2, STYLE_JOYLEFT = 1u << 3, STYLE_JOYRIGHT = 1u << 4 };
enum { ATTR_CONNECTED = 1u << 0, ATTR_WIRED = 1u << 1, ATTR_LEFT_CONN = 1u << 2, ATTR_LEFT_WIRED = 1u << 3, ATTR_RIGHT_CONN = 1u << 4, ATTR_RIGHT_WIRED = 1u << 5 };
enum { STICK_THRESH = 0x4000 };

typedef struct { int64_t sampling; uint64_t buttons; int32_t lx, ly, rx, ry; uint32_t attributes, _pad; } NpadState;
_Static_assert(sizeof(NpadState) == 0x28, "NpadState");

static _Atomic uint64_t g_btn, g_axes, g_latch, g_sc_btn, g_sc_axes, g_sc_latch, g_pad_btn[DM_PLAYERS], g_pad_axes[DM_PLAYERS], g_pad_latch[DM_PLAYERS];
static _Atomic int64_t g_sampling = 1 << 16;
static uint64_t pack(int32_t lx, int32_t ly, int32_t rx, int32_t ry) {
    return (uint64_t)(uint16_t)lx | (uint64_t)(uint16_t)ly << 16 | (uint64_t)(uint16_t)rx << 32 | (uint64_t)(uint16_t)ry << 48;
}

#define SHAKE_HZ 6.0
static const uint64_t SHAKE_CYCLE_NS = (uint64_t)(1e9 / SHAKE_HZ);
static _Atomic uint64_t g_shake_t0, g_shake_end;
static _Atomic int g_shake_on;
static void shake_note(void) {
    int on = ((atomic_load(&g_btn) | atomic_load(&g_sc_btn) | atomic_load(&g_pad_btn[0])) & DM_BTN_SHAKE) != 0;
    if (atomic_exchange(&g_shake_on, on) == on) return;
    uint64_t t = guest_ns();
    if (on) { if (t >= atomic_load(&g_shake_end)) atomic_store(&g_shake_t0, t); atomic_store(&g_shake_end, UINT64_MAX); }
    else { uint64_t t0 = atomic_load(&g_shake_t0); atomic_store(&g_shake_end, t0 + ((t - t0) / SHAKE_CYCLE_NS + 1) * SHAKE_CYCLE_NS); }
}
void dm_input_set(uint64_t buttons, int32_t lx, int32_t ly, int32_t rx, int32_t ry) {
    uint64_t old = atomic_exchange(&g_btn, buttons); atomic_fetch_or(&g_latch, buttons & ~old); atomic_store(&g_axes, pack(lx, ly, rx, ry));
    shake_note();
}
void dm_pad_set_player(int p, uint64_t buttons, int32_t lx, int32_t ly, int32_t rx, int32_t ry) {
    if (p < 0 || p >= DM_PLAYERS) return;
    uint64_t old = atomic_exchange(&g_pad_btn[p], buttons); atomic_fetch_or(&g_pad_latch[p], buttons & ~old); atomic_store(&g_pad_axes[p], pack(lx, ly, rx, ry));
    if (!p) shake_note();
}
void dm_pad_set(uint64_t buttons, int32_t lx, int32_t ly, int32_t rx, int32_t ry) { dm_pad_set_player(0, buttons, lx, ly, rx, ry); }
static void script_set(uint64_t buttons, int32_t lx, int32_t ly, int32_t rx, int32_t ry) {
    uint64_t old = atomic_exchange(&g_sc_btn, buttons); atomic_fetch_or(&g_sc_latch, buttons & ~old); atomic_store(&g_sc_axes, pack(lx, ly, rx, ry));
    shake_note();
}
uint64_t dm_input_samples(void) { return (uint64_t)atomic_load(&g_sampling); }

typedef struct { double at, hold; uint64_t btn; int32_t lx, ly, rx, ry; int frame, fhold, tap; float tx, ty; } key_step;
#define KEY_STEPS (1 << 20)
static key_step *g_steps; static int g_nsteps, g_cap_steps;
static int step_add(const key_step *st) {
    if (g_nsteps >= KEY_STEPS) return 0;
    if (g_nsteps == g_cap_steps) { int cap = g_cap_steps ? g_cap_steps * 2 : 64; key_step *n = realloc(g_steps, (size_t)cap * sizeof *n); if (!n) return 0; g_steps = n; g_cap_steps = cap; }
    g_steps[g_nsteps++] = *st; return 1;
}
static void *keys_main(void *arg) {
    (void)arg; struct timespec t0; clock_gettime(CLOCK_MONOTONIC, &t0); int touching = 0; int64_t released = -1;
    for (int i = 0; i < g_nsteps; i++) {
        const key_step *s = &g_steps[i];
        for (;;) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); double el = (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
                   if (s->frame ? (double)dm_guest_frame() >= s->at && (int64_t)dm_guest_frame() > released : el >= s->at) break; usleep(5000); }
        LOG("DM_KEYS: %s%.2f press %#llx (hold %.2f %s)", s->frame ? "frame " : "", s->at, (unsigned long long)s->btn, s->hold, s->fhold ? "frames" : "s");
        if (s->tap) { dm_touch t = {0, s->tx, s->ty}; dm_touch_script(&t, 1); } else if (touching) dm_touch_script(NULL, 0);
        script_set(s->btn, s->lx, s->ly, s->rx, s->ry);
        uint64_t end = 0;
        if (s->fhold) { end = (s->frame ? (uint64_t)s->at : dm_guest_frame()) + (uint64_t)s->hold; while (dm_guest_frame() < end) usleep(5000); }
        else usleep((useconds_t)(s->hold * 1e6));
        int next = s->fhold && i + 1 < g_nsteps && g_steps[i + 1].frame && g_steps[i + 1].at <= (double)end;
        touching = next && s->tap;
        if (!next) { script_set(0, 0, 0, 0, 0); if (s->tap) dm_touch_script(NULL, 0); released = (int64_t)dm_guest_frame(); }
    }
    return NULL;
}
static _Atomic int g_sync_on; static pthread_mutex_t g_sync_mu = PTHREAD_MUTEX_INITIALIZER;
static int g_sync_next, g_sync_held;
static int keys_framed(void) { for (int i = 0; i < g_nsteps; i++) if (!g_steps[i].frame || !g_steps[i].fhold) return 0; return g_nsteps > 0; }
static void keys_sync(void) {
    if (!atomic_load_explicit(&g_sync_on, memory_order_acquire)) return;
    pthread_mutex_lock(&g_sync_mu);
    uint64_t f = dm_guest_frame();
    for (;;) {
        if (g_sync_held) {
            const key_step *s = &g_steps[g_sync_next - 1]; uint64_t end = (uint64_t)s->at + (uint64_t)s->hold;
            if (end > f) break;
            if (g_sync_next < g_nsteps && g_steps[g_sync_next].at <= (double)end) {
                const key_step *n = &g_steps[g_sync_next++];
                if (n->tap) { dm_touch t = {0, n->tx, n->ty}; dm_touch_script(&t, 1); } else if (s->tap) dm_touch_script(NULL, 0);
                script_set(n->btn, n->lx, n->ly, n->rx, n->ry); continue;
            }
            script_set(0, 0, 0, 0, 0); if (s->tap) dm_touch_script(NULL, 0);
            g_sync_held = 0; continue;
        }
        if (g_sync_next >= g_nsteps) { atomic_store(&g_sync_on, 0); LOG("DM_KEYS: the %d frame steps were played, the last let go at frame %llu", g_nsteps, (unsigned long long)f); break; }
        const key_step *s = &g_steps[g_sync_next];
        if (s->at > (double)f) break;
        if (s->tap) { dm_touch t = {0, s->tx, s->ty}; dm_touch_script(&t, 1); }
        script_set(s->btn, s->lx, s->ly, s->rx, s->ry); g_sync_next++; g_sync_held = 1;
    }
    pthread_mutex_unlock(&g_sync_mu);
}
static const char *const key_names[16] = {"A", "B", "X", "Y", "LS", "RS", "L", "R", "ZL", "ZR", "Plus", "Minus", "Left", "Up", "Right", "Down"};
static void parse_keys(char *spec, key_step *st) {
    for (char *k = spec; k && *k;) { char *plus = strchr(k, '+'); if (plus) *plus = 0;
        int found = 0; for (int b = 0; b < 16; b++) if (!strcasecmp(k, key_names[b])) { st->btn |= 1ull << b; found = 1; }
        static const char *const sticks[4] = {"LLeft", "LUp", "LRight", "LDown"};
        static const int sx[4] = {-32767, 0, 32767, 0}, sy[4] = {0, 32767, 0, -32767};
        for (int d = 0; d < 4; d++) if (!strcasecmp(k, sticks[d])) { if (sx[d]) st->lx = sx[d]; if (sy[d]) st->ly = sy[d]; found = 1; }
        static const char *const rsticks[4] = {"RLeft", "RUp", "RRight", "RDown"};
        for (int d = 0; d < 4; d++) if (!strcasecmp(k, rsticks[d])) { if (sx[d]) st->rx = sx[d]; if (sy[d]) st->ry = sy[d]; found = 1; }
        float fx, fy; if ((k[0] == 'L' || k[0] == 'R') && strchr(k, '/') && sscanf(k + 1, "%f/%f", &fx, &fy) == 2 && fabsf(fx) <= 1 && fabsf(fy) <= 1) {
            int32_t vx = (int32_t)lrintf(fx * 32767), vy = (int32_t)lrintf(fy * 32767);
            if (k[0] == 'L') { st->lx = vx; st->ly = vy; } else { st->rx = vx; st->ry = vy; } found = 1; }
        float tx, ty; if ((k[0] == 'T' || k[0] == 't') && sscanf(k + 1, "%fx%f", &tx, &ty) == 2) { st->tap = 1; st->tx = tx; st->ty = ty; found = 1; }
        if (!strcasecmp(k, "Shake")) { st->btn |= DM_BTN_SHAKE; found = 1; }
        if (!found) LOG("[RED] DM_KEYS unknown key name: %s", k);
        k = plus ? plus + 1 : NULL; }
}
static int step_cmp(const void *a, const void *b) { double x = ((const key_step *)a)->at, y = ((const key_step *)b)->at; return (x > y) - (x < y); }
static void keys_parse(const char *spec) {
    char *buf = strdup(spec), *save = NULL; if (!buf) return;
    int full = 0;
    for (char *tok = strtok_r(buf, ", \t\r\n", &save); tok && !full; tok = strtok_r(NULL, ", \t\r\n", &save)) {
        char *c = strchr(tok, ':'); if (!c) { LOG("[RED] DM_KEYS step lacks 'seconds:': %s", tok); continue; }
        *c = 0; int fr = (tok[0] == 'f' || tok[0] == 'F'); key_step st = {atof(tok + fr), 0.15, 0}; st.frame = fr;
        int rep = 1; double every = 0;
        char *r = strchr(c + 1, '*'); if (r) { *r = 0; rep = atoi(r + 1); char *sl = strchr(r + 1, '/'); every = sl ? atof(sl + 1) : 1; if (rep < 1) rep = 1; }
        char *h = strchr(c + 1, '~'); if (h) { *h = 0; st.fhold = h[1] == 'f' || h[1] == 'F'; st.hold = atof(h + 1 + st.fhold); }
        parse_keys(c + 1, &st);
        for (int i = 0; i < rep && !full; i++) { key_step one = st; one.at = st.at + i * every; if (!step_add(&one)) { LOG("[RED] DM_KEYS: more than %d steps, the rest is dropped", KEY_STEPS); full = 1; } }
    }
    free(buf);
    int mixed = 0; for (int i = 1; i < g_nsteps; i++) mixed |= g_steps[i].frame != g_steps[0].frame;
    if (!mixed && g_nsteps > 1) mergesort(g_steps, (size_t)g_nsteps, sizeof *g_steps, step_cmp);
}
static char *keys_file(const char *path) {
    FILE *f = fopen(path, "rb"); if (!f) return NULL;
    char *text = NULL; size_t len = 0, cap = 0, n;
    do { if (len + 65536 + 1 > cap) { cap = cap ? cap * 2 : 1 << 20; char *t = realloc(text, cap); if (!t) { free(text); fclose(f); return NULL; } text = t; }
         n = fread(text + len, 1, cap - len - 1, f); len += n; } while (n);
    fclose(f); text[len] = 0; return text;
}
static void ctl_start(void);
static void rec_start(void);
void hid_keys_init(void) {
    ctl_start();
    rec_start();
    const char *e = getenv("DM_KEYS"); if (!e || !*e) return;
    if (e[0] == '@') {
        char *text = keys_file(e + 1); if (!text) { LOG("[RED] DM_KEYS: cannot read %s", e + 1); return; }
        keys_parse(text); free(text);
    } else keys_parse(e);
    if (keys_framed()) { atomic_store(&g_sync_on, 1); LOG("DM_KEYS: %d steps, all in frames: applied at the game's input reads of their frames", g_nsteps); return; }
    pthread_t th; if (g_nsteps && !pthread_create(&th, NULL, keys_main, NULL)) { pthread_detach(th); LOG("DM_KEYS: %d steps", g_nsteps); }
}

static void *ctl_main(void *arg) {
    const char *path = arg; long off = 0; char line[1100];
    for (;;) {
        FILE *f = fopen(path, "r");
        if (f && !fseek(f, off, SEEK_SET)) {
            while (fgets(line, sizeof line, f)) {
                size_t n = strlen(line); if (!n || line[n - 1] != '\n') break;
                off += (long)n; line[--n] = 0; if (n && line[n - 1] == '\r') line[--n] = 0;
                if (!n || line[0] == '#') continue;
                if (!strcmp(line, "dump")) { crash_dump_now("DM_CTL dump"); continue; }
                if (!strncmp(line, "shot ", 5)) { dm_gfx_shot_at(1, line + 5); LOG("DM_CTL: screenshot of the next frame → %s", line + 5); continue; }
                if (!strncmp(line, "texlist ", 8)) { void dm_gfx_texlist(int target); dm_gfx_texlist(atoi(line + 8)); continue; }
                if (!strncmp(line, "texraw ", 7)) { void dm_gfx_texraw(uint64_t serial, const char *path); char *e2; uint64_t s = strtoull(line + 7, &e2, 10); while (*e2 == ' ') e2++; dm_gfx_texraw(s, e2); continue; }
                if (!strncmp(line, "peek ", 5)) { void dm_ctl_peek(uint64_t a, int n); char *e2; uint64_t a = strtoull(line + 5, &e2, 16); dm_ctl_peek(a, (int)strtol(e2, NULL, 0)); continue; }
                if (!strncmp(line, "memdump ", 8)) { void dm_ctl_memdump(uint64_t a, uint64_t n, const char *path); char *e2, *e3; uint64_t a = strtoull(line + 8, &e2, 16), n = strtoull(e2, &e3, 0);
                                                     while (*e3 == ' ') e3++; dm_ctl_memdump(a, n, e3); continue; }
                if (!strncmp(line, "cvari ", 6)) { void dm_ctl_cvar_i32(uint64_t off, int32_t v); char *e2; uint64_t off = strtoull(line + 6, &e2, 16); dm_ctl_cvar_i32(off, (int32_t)strtol(e2, NULL, 0)); continue; }
                if (!strncmp(line, "cvarf ", 6)) { void dm_ctl_cvar_f32(uint64_t off, float v); char *e2; uint64_t off = strtoull(line + 6, &e2, 16); dm_ctl_cvar_f32(off, strtof(e2, NULL)); continue; }
                if (!strncmp(line, "env ", 4)) { char *eq = strchr(line + 4, '='); if (eq) { *eq = 0; setenv(line + 4, eq + 1, 1); LOG("DM_CTL: env %s=%s", line + 4, eq + 1); } continue; }
                if (!strncmp(line, "texdump ", 8)) { void dm_gfx_texdump_next(const char *path); dm_gfx_texdump_next(line + 8); LOG("DM_CTL: colour targets of the next frame → %s (as DM_NVN_TEXDUMP)", line + 8); continue; }
                if (!strncmp(line, "passdump ", 9)) { void dm_gfx_passdump_next(long n); dm_gfx_passdump_next(atol(line + 9)); LOG("DM_CTL: passes of the next %ld frames (as DM_NVN_PASSDUMP)", atol(line + 9)); continue; }
                if (!strncmp(line, "drag ", 5)) {
                    float x0, y0, x1, y1, s = 0.3f; if (sscanf(line + 5, "%f %f %f %f %f", &x0, &y0, &x1, &y1, &s) < 4) { LOG("[RED] DM_CTL drag wants x0 y0 x1 y1 [seconds]"); continue; }
                    int n = (int)(s / 0.008f); if (n < 2) n = 2;
                    LOG("DM_CTL: drag (%.3f,%.3f) → (%.3f,%.3f) over %.2f s", x0, y0, x1, y1, s);
                    for (int i = 0; i <= n; i++) { dm_touch t = {0, x0 + (x1 - x0) * (float)i / (float)n, y0 + (y1 - y0) * (float)i / (float)n}; dm_touch_script(&t, 1); usleep((useconds_t)(s * 1e6f / (float)n)); }
                    dm_touch_script(NULL, 0); continue;
                }
                key_step st = {0, 0.15, 0}; char *h = strchr(line, '~'); if (h) { *h = 0; st.hold = atof(h + 1); }
                parse_keys(line, &st);
                LOG("DM_CTL: press %#llx (hold %.2f s)", (unsigned long long)st.btn, st.hold);
                if (st.tap) { dm_touch t = {0, st.tx, st.ty}; dm_touch_script(&t, 1); }
                script_set(st.btn, st.lx, st.ly, st.rx, st.ry); usleep((useconds_t)(st.hold * 1e6)); script_set(0, 0, 0, 0, 0);
                if (st.tap) dm_touch_script(NULL, 0);
            }
        }
        if (f) fclose(f);
        usleep(100000);
    }
    return NULL;
}
static void ctl_start(void) {
    const char *e = getenv("DM_CTL"); if (!e || !*e) return;
    pthread_t th; if (!pthread_create(&th, NULL, ctl_main, strdup(e))) { pthread_detach(th); LOG("DM_CTL: reading key presses and screenshot requests from %s", e); }
}

// Pro Controller mapping: Player 1 is always connected on No1 (id 0); further players map directly to No2..No8 (ids 1..7).
static uint32_t connected_id(void) { return NPAD_NO1; }
uint32_t hle_hid_connected_npad(void) { return NPAD_NO1; }
uint32_t dm_player_npad(int p) { return (uint32_t)(p >= 0 && p < DM_PLAYERS ? p : 0); }

static _Atomic uint32_t g_players = 1, g_ids_supported = 0x3ff;
static uint32_t id_bit(uint32_t id) { return id < DM_PLAYERS ? 1u << id : id == NPAD_HANDHELD ? 1u << 8 : id == 0x10 ? 1u << 9 : 0; }
static int reported(int p) {
    if (p < 0 || p >= DM_PLAYERS) return 0;
    return ((atomic_load(&g_players) >> p) & 1) && ((atomic_load(&g_ids_supported) >> p) & 1);
}

static int player_of(uint32_t id) {
    if (id < DM_PLAYERS && reported((int)id)) return (int)id;
    return -1;
}
static uint32_t style_for_player(uint32_t id, int p) {
    (void)id;
    return p < 0 ? 0 : STYLE_FULLKEY;
}
static uint32_t style_of(uint32_t id) { return style_for_player(id, player_of(id)); }
static int32_t ax(uint64_t v, int i) { return (int16_t)(v >> (16 * i)); }
static int32_t pick(int32_t a, int32_t b) { return (a < 0 ? -a : a) >= (b < 0 ? -b : b) ? a : b; }

static void *_Atomic g_style_ev[DM_PLAYERS];
static void style_changed(uint32_t id) { void *ev = id < DM_PLAYERS ? atomic_load(&g_style_ev[id]) : NULL; if (ev) hle_signal_bound_system_event(ev); }

void dm_pad_players(uint32_t mask) {
    mask = (mask | 1u) & ((1u << DM_PLAYERS) - 1);
    uint32_t old = atomic_exchange(&g_players, mask);
    for (int p = 1; p < DM_PLAYERS; p++) if ((old ^ mask) >> p & 1) {
        if (!(mask >> p & 1)) dm_pad_set_player(p, 0, 0, 0, 0, 0);
        if (atomic_load(&g_ids_supported) & id_bit((uint32_t)p)) style_changed((uint32_t)p);
    }
}

// DM_REC
enum { REC_STICK_STEP = 2, REC_TOUCH_STEP = 5, REC_FLUSH_MS = 250 };
typedef struct { uint64_t btn; int32_t q[4]; int touch; int32_t tx, ty; } rec_state;
static _Atomic int g_rec_on;
static pthread_mutex_t g_rec_mu = PTHREAD_MUTEX_INITIALIZER, g_rec_io = PTHREAD_MUTEX_INITIALIZER;
static rec_state g_rec_cur; static uint64_t g_rec_from;
static rec_state g_rec_frame; static uint64_t g_rec_at; static int g_rec_have;
static int g_rec_touch, g_rec_tlatch; static float g_rec_tx, g_rec_ty;
static char *g_rec_buf; static size_t g_rec_len, g_rec_cap;
static FILE *g_rec_file;
extern void (*_Atomic dm_touch_observer)(const dm_touch *t, int n);
static int rec_idle(const rec_state *s) { return !s->btn && !s->q[0] && !s->q[1] && !s->q[2] && !s->q[3] && !s->touch; }
static int rec_same(const rec_state *a, const rec_state *b) {
    if (a->btn != b->btn || a->touch != b->touch) return 0;
    if (a->touch && (abs(a->tx - b->tx) >= REC_TOUCH_STEP || abs(a->ty - b->ty) >= REC_TOUCH_STEP)) return 0;
    for (int i = 0; i < 4; i++) if (!a->q[i] != !b->q[i] || abs(a->q[i] - b->q[i]) >= REC_STICK_STEP) return 0;
    return 1;
}
static int rec_keys(const rec_state *s, char *out, size_t cap) {
    int n = 0;
    #define PUT(fmt, ...) do { int w_ = snprintf(out + n, cap - (size_t)n, "%s" fmt, n ? "+" : "", ##__VA_ARGS__); if (w_ > 0 && (size_t)(n + w_) < cap) n += w_; } while (0)
    for (int b = 0; b < 16; b++) if (s->btn >> b & 1) PUT("%s", key_names[b]);
    if (s->btn & DM_BTN_SHAKE) PUT("Shake");
    if (s->q[0] || s->q[1]) PUT("L%.2f/%.2f", s->q[0] / 100.0, s->q[1] / 100.0);
    if (s->q[2] || s->q[3]) PUT("R%.2f/%.2f", s->q[2] / 100.0, s->q[3] / 100.0);
    if (s->touch) PUT("T%.3fx%.3f", s->tx / 1000.0, s->ty / 1000.0);
    #undef PUT
    return n;
}
static void rec_step(const rec_state *s, uint64_t from, uint64_t to) {
    char keys[160], line[200]; rec_keys(s, keys, sizeof keys);
    int n = snprintf(line, sizeof line, "f%llu:%s~f%llu,\n", (unsigned long long)from, keys, (unsigned long long)(to - from));
    if (n <= 0 || (size_t)n >= sizeof line) return;
    if (g_rec_len + (size_t)n > g_rec_cap) { size_t cap = g_rec_cap ? g_rec_cap * 2 : 4096; while (cap < g_rec_len + (size_t)n) cap *= 2; char *b = realloc(g_rec_buf, cap); if (!b) return; g_rec_buf = b; g_rec_cap = cap; }
    memcpy(g_rec_buf + g_rec_len, line, (size_t)n); g_rec_len += (size_t)n;
}
static void rec_frame_done(void) {
    if (!g_rec_have) return;
    if (!rec_same(&g_rec_frame, &g_rec_cur)) {
        if (!rec_idle(&g_rec_cur)) rec_step(&g_rec_cur, g_rec_from, g_rec_at);
        g_rec_cur = g_rec_frame; g_rec_from = g_rec_at;
    }
    g_rec_have = 0;
}
static void rec_read(uint64_t btn, const int32_t a[4]) {
    rec_state s = {btn};
    for (int i = 0; i < 4; i++) { long q = lrint(a[i] * 100.0 / DM_STICK_MAX); s.q[i] = (int32_t)(q < -100 ? -100 : q > 100 ? 100 : q); }
    pthread_mutex_lock(&g_rec_mu);
    uint64_t f = dm_guest_frame();
    if (g_rec_have && f != g_rec_at) rec_frame_done();
    if ((s.touch = g_rec_touch || g_rec_tlatch)) { s.tx = (int32_t)lrintf(g_rec_tx * 1000); s.ty = (int32_t)lrintf(g_rec_ty * 1000); }
    g_rec_tlatch = 0;
    if (!g_rec_have) { g_rec_frame = s; g_rec_at = f; g_rec_have = 1; }
    else { s.btn |= g_rec_frame.btn; g_rec_frame = s; }
    pthread_mutex_unlock(&g_rec_mu);
}
static void rec_touch(const dm_touch *t, int n) {
    pthread_mutex_lock(&g_rec_mu); g_rec_touch = n > 0; if (n > 0) { g_rec_tx = t[0].x; g_rec_ty = t[0].y; g_rec_tlatch = 1; } pthread_mutex_unlock(&g_rec_mu);
}
static void rec_flush(void) {
    pthread_mutex_lock(&g_rec_io);
    pthread_mutex_lock(&g_rec_mu); char *b = g_rec_buf; size_t n = g_rec_len; g_rec_buf = NULL; g_rec_len = g_rec_cap = 0; pthread_mutex_unlock(&g_rec_mu);
    if (n && g_rec_file) { fwrite(b, 1, n, g_rec_file); fflush(g_rec_file); }
    free(b);
    pthread_mutex_unlock(&g_rec_io);
}
void dm_rec_finish(void) {
    if (!atomic_load(&g_rec_on)) return;
    pthread_mutex_lock(&g_rec_mu);
    uint64_t last = g_rec_have ? g_rec_at + 1 : 0; rec_frame_done();
    uint64_t f = dm_guest_frame(); if (f < last) f = last;
    if (!rec_idle(&g_rec_cur)) rec_step(&g_rec_cur, g_rec_from, f);
    g_rec_cur = (rec_state){0}; g_rec_from = f;
    pthread_mutex_unlock(&g_rec_mu);
    rec_flush();
}
static void *rec_main(void *arg) { (void)arg; for (;;) { usleep(REC_FLUSH_MS * 1000); rec_flush(); } return NULL; }
static int rec_open(const char *path) {
    if (!(g_rec_file = fopen(path, "w"))) { LOG("[RED] DM_REC: cannot write %s", path); return 0; }
    atomic_store(&dm_touch_observer, rec_touch);
    atomic_store(&g_rec_on, 1);
    return 1;
}
static void rec_start(void) {
    const char *e = getenv("DM_REC"); if (!e || !*e || !rec_open(e)) return;
    pthread_t th; if (!pthread_create(&th, NULL, rec_main, NULL)) pthread_detach(th);
    atexit(dm_rec_finish);
    LOG("DM_REC: recording player 1's input to %s as DM_KEYS steps in presented frames", e);
}

static void fill(NpadState *s, uint32_t id, uint32_t style) {
    memset(s, 0, sizeof *s);
    s->sampling = atomic_fetch_add(&g_sampling, 1) + 1;
    int pl = player_of(id);
    if (pl < 0 || !(style_for_player(id, pl) & style)) return;

    uint64_t b = 0; int32_t a[4] = {0};
    if (pl == 0) {
        keys_sync();
        b = atomic_load(&g_btn) | atomic_exchange(&g_latch, 0);
        uint64_t k = atomic_load(&g_axes); for (int i = 0; i < 4; i
