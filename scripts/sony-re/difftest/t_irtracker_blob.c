#include "sony.h"
#include "psvr2_ld.h"
typedef void (*fn_t)(const uint8_t *rec, char ext, void *out, int *count, void *sum);
static int feq(float a, float b, int tol) { int32_t ia, ib; memcpy(&ia, &a, 4); memcpy(&ib, &b, 4); return abs(ia - ib) <= tol; }
int main(int argc, char **argv) {
    sony_load(argc > 1 ? argv[1] : "sony.dll");
    fn_t f = (fn_t)SONY(0x180184900);
    int bad = 0, angle_ulp = 0, n = 300000;
    static psvr2_blob_set a, b;
    for (int i = 0; i < n; i++) {
        uint8_t rec[36];
        for (int k = 0; k < 36; k++) rec[k] = (uint8_t)rnd();
        const uint8_t kinds[] = {1, 2, 5, 3, 0};
        rec[0] = kinds[rnd() % 5];
        int16_t x0 = rnd() % 1000, y0 = rnd() % 1000, x1 = x0 + rnd() % 12, y1 = y0 + rnd() % 12;
        memcpy(rec + 4, &x0, 2); memcpy(rec + 6, &x1, 2); memcpy(rec + 8, &y0, 2); memcpy(rec + 10, &y1, 2);
        uint32_t m00 = rnd() % 8 == 0 ? 0 : 1 + rnd() % 40000;
        memcpy(rec + 12, &m00, 4);
        for (int k = 0; k < 5; k++) { uint32_t v = rnd() % (m00 * 30 + 1); memcpy(rec + 16 + 4 * k, &v, 4); }
        int ext = rnd() & 1, ca = 0, cb = 0;
        psvr2_blob_summary sa, sb;
        memset(&a, 0, sizeof a); memset(&b, 0, sizeof b); memset(&sa, 0, sizeof sa); memset(&sb, 0, sizeof sb);
        f(rec, (char)ext, &a, &ca, &sa);
        psvr2_ld_decode_blob(rec, ext, &b, &cb, &sb);
        int ok = ca == cb && memcmp(&sa, &sb, offsetof(psvr2_blob_summary, x)) == 0 && sa.id == sb.id && sa.kind == sb.kind &&
                 feq(sa.x, sb.x, 0) && feq(sa.y, sb.y, 0) && feq(sa.major, sb.major, 0) && feq(sa.minor, sb.minor, 0);
        if (ca && cb) {
            ok = ok && feq(a.x[0], b.x[0], 0) && feq(a.y[0], b.y[0], 0) && feq(a.major[0], b.major[0], 0) &&
                 feq(a.minor[0], b.minor[0], 0) && a.link[0] == b.link[0] && feq(a.mass[0], b.mass[0], 0) &&
                 a.w[0] == b.w[0] && a.h[0] == b.h[0] && feq(a.weight[0], b.weight[0], 0) && a.id[0] == b.id[0] &&
                 a.flags[0] == b.flags[0];
            if (!feq(a.angle[0], b.angle[0], 0)) { if (feq(a.angle[0], b.angle[0], 2)) angle_ulp++; else ok = 0; }
        }
        if (!ok && bad++ < 4) printf("mismatch i=%d kind=%d ext=%d c=%d/%d x %g/%g maj %g/%g ang %g/%g fl %x/%x\n", i, rec[0], ext, ca, cb,
                                     a.x[0], b.x[0], a.major[0], b.major[0], a.angle[0], b.angle[0], a.flags[0], b.flags[0]);
    }
    printf("blob: %d/%d mismatches, %d angles differ by <=2 ulp (libm atanf)\n", bad, n, angle_ulp);
    return bad != 0;
}
