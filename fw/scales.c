/* Scales and tunings, see scales.h. The usual scales in steps of 12, the Arabic maqamat in steps
 * of 24 (quarter tones), Partch's in steps of his 43-tone just scale; tunings as ratios, cents or
 * equal divisions, some with an interval of repetition other than the octave (Bohlen-Pierce 3:1,
 * Carlos alpha / beta / gamma). A scale uses the selected tuning when it has the scale's number of
 * steps, else the equal division of the octave into that many; the chromatic scales play every
 * step of whatever tuning is selected. */
#include "scales.h"

#include "dmath.h"

struct scale {
    const char* name;
    uint8_t div;       /* steps per octave the degrees count in */
    uint8_t n;         /* degrees; 0: every step of the selected tuning (chromatic) */
    uint8_t step[8];
};

static const struct scale g_scales[] = {
    /* seven notes */
    {"major", 12, 7, {0, 2, 4, 5, 7, 9, 11}},
    {"minor", 12, 7, {0, 2, 3, 5, 7, 8, 10}},
    {"ionian", 12, 7, {0, 2, 4, 5, 7, 9, 11}},
    {"dorian", 12, 7, {0, 2, 3, 5, 7, 9, 10}},
    {"phrygian", 12, 7, {0, 1, 3, 5, 7, 8, 10}},
    {"lydian", 12, 7, {0, 2, 4, 6, 7, 9, 11}},
    {"mixolydian", 12, 7, {0, 2, 4, 5, 7, 9, 10}},
    {"aeolian", 12, 7, {0, 2, 3, 5, 7, 8, 10}},
    {"locrian", 12, 7, {0, 1, 3, 5, 6, 8, 10}},
    {"harmonicMinor", 12, 7, {0, 2, 3, 5, 7, 8, 11}},
    {"harmonicMajor", 12, 7, {0, 2, 4, 5, 7, 8, 11}},
    {"melodicMinor", 12, 7, {0, 2, 3, 5, 7, 9, 11}},
    {"melodicMinorDesc", 12, 7, {0, 2, 3, 5, 7, 8, 10}},
    {"melodicMajor", 12, 7, {0, 2, 4, 5, 7, 8, 10}},
    {"bartok", 12, 7, {0, 2, 4, 5, 7, 8, 10}},
    {"hindu", 12, 7, {0, 2, 4, 5, 7, 8, 10}},
    {"superLocrian", 12, 7, {0, 1, 3, 4, 6, 8, 10}},
    {"romanianMinor", 12, 7, {0, 2, 3, 6, 7, 9, 10}},
    {"hungarianMinor", 12, 7, {0, 2, 3, 6, 7, 8, 11}},
    {"neapolitanMinor", 12, 7, {0, 1, 3, 5, 7, 8, 11}},
    {"neapolitanMajor", 12, 7, {0, 1, 3, 5, 7, 9, 11}},
    {"enigmatic", 12, 7, {0, 1, 4, 6, 8, 10, 11}},
    {"spanish", 12, 7, {0, 1, 4, 5, 7, 8, 10}},
    {"leadingWhole", 12, 7, {0, 2, 4, 6, 8, 10, 11}},
    {"lydianMinor", 12, 7, {0, 2, 4, 6, 7, 8, 10}},
    {"locrianMajor", 12, 7, {0, 2, 4, 5, 6, 8, 10}},
    {"todi", 12, 7, {0, 1, 3, 6, 7, 8, 11}},
    {"purvi", 12, 7, {0, 1, 4, 6, 7, 8, 11}},
    {"marva", 12, 7, {0, 1, 4, 6, 7, 9, 11}},
    {"bhairav", 12, 7, {0, 1, 4, 5, 7, 8, 11}},
    {"ahirbhairav", 12, 7, {0, 1, 4, 5, 7, 9, 10}},
    /* five notes */
    {"minorPentatonic", 12, 5, {0, 3, 5, 7, 10}},
    {"majorPentatonic", 12, 5, {0, 2, 4, 7, 9}},
    {"ritusen", 12, 5, {0, 2, 5, 7, 9}},
    {"egyptian", 12, 5, {0, 2, 5, 7, 10}},
    {"kumoi", 12, 5, {0, 2, 3, 7, 9}},
    {"hirajoshi", 12, 5, {0, 2, 3, 7, 8}},
    {"iwato", 12, 5, {0, 1, 5, 6, 10}},
    {"chinese", 12, 5, {0, 4, 6, 7, 11}},
    {"indian", 12, 5, {0, 4, 5, 7, 10}},
    {"pelog", 12, 5, {0, 1, 3, 7, 8}},
    {"prometheus", 12, 5, {0, 2, 4, 6, 11}},
    {"scriabin", 12, 5, {0, 1, 4, 7, 9}},
    {"gong", 12, 5, {0, 2, 4, 7, 9}},
    {"shang", 12, 5, {0, 2, 5, 7, 10}},
    {"jiao", 12, 5, {0, 3, 5, 8, 10}},
    {"zhi", 12, 5, {0, 2, 5, 7, 9}},
    {"yu", 12, 5, {0, 3, 5, 7, 10}},
    /* six notes */
    {"blues", 12, 6, {0, 3, 5, 6, 7, 10}},
    {"whole", 12, 6, {0, 2, 4, 6, 8, 10}},
    {"augmented", 12, 6, {0, 3, 4, 7, 8, 11}},
    {"augmented2", 12, 6, {0, 1, 4, 5, 8, 9}},
    {"hexMajor7", 12, 6, {0, 2, 4, 7, 9, 11}},
    {"hexDorian", 12, 6, {0, 2, 3, 5, 7, 10}},
    {"hexPhrygian", 12, 6, {0, 1, 3, 5, 8, 10}},
    {"hexSus", 12, 6, {0, 2, 5, 7, 9, 10}},
    {"hexMajor6", 12, 6, {0, 2, 4, 5, 7, 9}},
    {"hexAeolian", 12, 6, {0, 3, 5, 7, 8, 10}},
    /* eight, twelve */
    {"diminished", 12, 8, {0, 1, 3, 4, 6, 7, 9, 10}},
    {"diminished2", 12, 8, {0, 2, 3, 5, 6, 8, 9, 11}},
    {"chromatic", 12, 0, {0}},
    /* Partch's otonalities and utonalities, in steps of his 43 */
    {"partch_o1", 43, 6, {0, 8, 14, 20, 25, 34}},
    {"partch_o2", 43, 6, {0, 7, 13, 18, 27, 35}},
    {"partch_o3", 43, 6, {0, 6, 12, 21, 29, 36}},
    {"partch_o4", 43, 6, {0, 5, 15, 23, 30, 37}},
    {"partch_o5", 43, 6, {0, 10, 18, 25, 31, 38}},
    {"partch_o6", 43, 6, {0, 9, 16, 22, 28, 33}},
    {"partch_u1", 43, 6, {0, 9, 18, 23, 29, 35}},
    {"partch_u2", 43, 6, {0, 8, 16, 25, 30, 36}},
    {"partch_u3", 43, 6, {0, 7, 14, 22, 31, 37}},
    {"partch_u4", 43, 6, {0, 6, 13, 20, 28, 38}},
    {"partch_u5", 43, 6, {0, 5, 12, 18, 25, 33}},
    {"partch_u6", 43, 6, {0, 10, 15, 21, 27, 34}},
    /* maqamat, in quarter tones */
    {"chromatic24", 24, 0, {0}},
    {"ajam", 24, 7, {0, 4, 8, 10, 14, 18, 22}},
    {"jiharkah", 24, 7, {0, 4, 8, 10, 14, 18, 21}},
    {"shawqAfza", 24, 7, {0, 4, 8, 10, 14, 16, 22}},
    {"sikah", 24, 7, {0, 3, 7, 11, 14, 17, 21}},
    {"sikahDesc", 24, 7, {0, 3, 7, 11, 13, 17, 21}},
    {"huzam", 24, 7, {0, 3, 7, 9, 15, 17, 21}},
    {"iraq", 24, 7, {0, 3, 7, 10, 13, 17, 21}},
    {"bastanikar", 24, 7, {0, 3, 7, 10, 13, 15, 21}},
    {"mustar", 24, 7, {0, 5, 7, 11, 13, 17, 21}},
    {"bayati", 24, 7, {0, 3, 6, 10, 14, 16, 20}},
    {"karjighar", 24, 7, {0, 3, 6, 10, 12, 18, 20}},
    {"husseini", 24, 7, {0, 3, 6, 10, 14, 17, 21}},
    {"nahawand", 24, 7, {0, 4, 6, 10, 14, 16, 22}},
    {"nahawandDesc", 24, 7, {0, 4, 6, 10, 14, 16, 20}},
    {"farahfaza", 24, 7, {0, 4, 6, 10, 14, 16, 20}},
    {"murassah", 24, 7, {0, 4, 6, 10, 12, 18, 20}},
    {"ushaqMashri", 24, 7, {0, 4, 6, 10, 14, 17, 21}},
    {"rast", 24, 7, {0, 4, 7, 10, 14, 18, 21}},
    {"rastDesc", 24, 7, {0, 4, 7, 10, 14, 18, 20}},
    {"suznak", 24, 7, {0, 4, 7, 10, 14, 16, 22}},
    {"nairuz", 24, 7, {0, 4, 7, 10, 14, 17, 20}},
    {"yakah", 24, 7, {0, 4, 7, 10, 14, 18, 21}},
    {"yakahDesc", 24, 7, {0, 4, 7, 10, 14, 18, 20}},
    {"mahur", 24, 7, {0, 4, 7, 10, 14, 18, 22}},
    {"hijaz", 24, 7, {0, 2, 8, 10, 14, 17, 20}},
    {"hijazDesc", 24, 7, {0, 2, 8, 10, 14, 16, 20}},
    {"zanjaran", 24, 7, {0, 2, 8, 10, 14, 18, 20}},
    {"hijazKar", 24, 7, {0, 2, 8, 10, 14, 16, 22}},
    {"saba", 24, 7, {0, 3, 6, 8, 12, 16, 20}},
    {"zamzam", 24, 7, {0, 2, 6, 8, 14, 16, 20}},
    {"kurd", 24, 7, {0, 2, 6, 10, 14, 16, 20}},
    {"kijazKarKurd", 24, 7, {0, 2, 8, 10, 14, 16, 22}},
    {"nawaAthar", 24, 7, {0, 4, 6, 12, 14, 16, 22}},
    {"nikriz", 24, 7, {0, 4, 6, 12, 14, 18, 20}},
    {"atharKurd", 24, 7, {0, 2, 6, 12, 14, 16, 22}},
};
#define NSCALES ((int)(sizeof(g_scales) / sizeof(g_scales[0])))

/* tunings: kind R ratios, S semitones, E an equal division (n steps of 'step' semitones) */
enum { T_RATIO, T_SEMIS, T_EQUAL };
struct tuning {
    const char* name;
    uint8_t kind, n;
    double rep;            /* the interval of repetition, as a ratio */
    const double* v;       /* ratios / semitones; T_EQUAL: v[0] = the step in semitones */
};

#define R(...) (const double[]){__VA_ARGS__}
static const struct tuning g_tunings[] = {
    {"et12", T_EQUAL, 12, 2, R(1.0)},
    {"just", T_RATIO, 12, 2, R(1, 16. / 15, 9. / 8, 6. / 5, 5. / 4, 4. / 3, 45. / 32, 3. / 2, 8. / 5, 5. / 3, 9. / 5, 15. / 8)},
    {"pythagorean", T_RATIO, 12, 2,
     R(1, 256. / 243, 9. / 8, 32. / 27, 81. / 64, 4. / 3, 729. / 512, 3. / 2, 128. / 81, 27. / 16, 16. / 9, 243. / 128)},
    {"sept1", T_RATIO, 12, 2, R(1, 16. / 15, 9. / 8, 6. / 5, 5. / 4, 4. / 3, 7. / 5, 3. / 2, 8. / 5, 5. / 3, 9. / 5, 15. / 8)},
    {"sept2", T_RATIO, 12, 2, R(1, 16. / 15, 9. / 8, 6. / 5, 5. / 4, 4. / 3, 7. / 5, 3. / 2, 8. / 5, 5. / 3, 7. / 4, 15. / 8)},
    {"mean4", T_SEMIS, 12, 2, R(0, 0.755, 1.93, 3.105, 3.86, 5.035, 5.79, 6.965, 7.72, 8.895, 10.07, 10.82)},
    {"mean5", T_SEMIS, 12, 2, R(0, 0.804, 1.944, 3.084, 3.888, 5.028, 5.832, 6.972, 7.776, 8.916, 10.056, 10.86)},
    {"mean6", T_SEMIS, 12, 2, R(0, 0.86, 1.96, 3.06, 3.92, 5.02, 5.88, 6.98, 7.84, 8.94, 10.04, 10.9)},
    {"kirnberger", T_RATIO, 12, 2,
     R(1, 256. / 243, 1.118033988749895, 32. / 27, 5. / 4, 4. / 3, 45. / 32, 1.4953487812212205, 128. / 81,
       1.6718507624410550, 16. / 9, 15. / 8)},
    {"werckmeister", T_SEMIS, 12, 2, R(0, 0.92, 1.93, 2.94, 3.915, 4.98, 5.9, 6.965, 7.93, 8.895, 9.96, 10.935)},
    {"vallotti", T_SEMIS, 12, 2,
     R(0, 0.94135, 1.9609, 2.98045, 3.92180, 5.01955, 5.9218, 6.98045, 7.9609, 8.94135, 10, 10.90225)},
    {"young", T_SEMIS, 12, 2, R(0, 0.9, 1.96, 2.94, 3.92, 4.98, 5.88, 6.98, 7.92, 8.94, 9.96, 10.9)},
    {"reinhard", T_RATIO, 12, 2,
     R(1, 14. / 13, 13. / 12, 16. / 13, 13. / 10, 18. / 13, 13. / 9, 20. / 13, 13. / 8, 22. / 13, 13. / 7, 208. / 105)},
    {"wcHarm", T_RATIO, 12, 2,
     R(1, 17. / 16, 9. / 8, 19. / 16, 5. / 4, 21. / 16, 11. / 8, 3. / 2, 13. / 8, 27. / 16, 7. / 4, 15. / 8)},
    {"wcSJ", T_RATIO, 12, 2, R(1, 17. / 16, 9. / 8, 6. / 5, 5. / 4, 4. / 3, 11. / 8, 3. / 2, 13. / 8, 5. / 3, 7. / 4, 15. / 8)},
    {"lu", T_RATIO, 12, 2,
     R(1, 2187. / 2048, 9. / 8, 19683. / 16384, 81. / 64, 177147. / 131072, 729. / 512, 3. / 2, 6561. / 4096, 27. / 16,
       59049. / 32768, 243. / 128)},
    {"et19", T_EQUAL, 19, 2, R(12. / 19)},
    {"et22", T_EQUAL, 22, 2, R(6. / 11)},
    {"et24", T_EQUAL, 24, 2, R(0.5)},
    {"et31", T_EQUAL, 31, 2, R(12. / 31)},
    {"et41", T_EQUAL, 41, 2, R(12. / 41)},
    {"et53", T_EQUAL, 53, 2, R(12. / 53)},
    {"johnston", T_RATIO, 25, 2,
     R(1, 25. / 24, 135. / 128, 16. / 15, 10. / 9, 9. / 8, 75. / 64, 6. / 5, 5. / 4, 81. / 64, 32. / 25, 4. / 3, 27. / 20,
       45. / 32, 36. / 25, 3. / 2, 25. / 16, 8. / 5, 5. / 3, 27. / 16, 225. / 128, 16. / 9, 9. / 5, 15. / 8, 48. / 25)},
    {"partch", T_RATIO, 43, 2,
     R(1, 81. / 80, 33. / 32, 21. / 20, 16. / 15, 12. / 11, 11. / 10, 10. / 9, 9. / 8, 8. / 7, 7. / 6, 32. / 27, 6. / 5,
       11. / 9, 5. / 4, 14. / 11, 9. / 7, 21. / 16, 4. / 3, 27. / 20, 11. / 8, 7. / 5, 10. / 7, 16. / 11, 40. / 27, 3. / 2,
       32. / 21, 14. / 9, 11. / 7, 8. / 5, 18. / 11, 5. / 3, 27. / 16, 12. / 7, 7. / 4, 16. / 9, 9. / 5, 20. / 11, 11. / 6,
       15. / 8, 40. / 21, 64. / 33, 160. / 81)},
    {"catler", T_RATIO, 24, 2,
     R(1, 33. / 32, 16. / 15, 9. / 8, 8. / 7, 7. / 6, 6. / 5, 128. / 105, 16. / 13, 5. / 4, 21. / 16, 4. / 3, 11. / 8,
       45. / 32, 16. / 11, 3. / 2, 8. / 5, 13. / 8, 5. / 3, 27. / 16, 7. / 4, 16. / 9, 24. / 13, 15. / 8)},
    {"chalmers", T_RATIO, 19, 2,
     R(1, 21. / 20, 16. / 15, 9. / 8, 7. / 6, 6. / 5, 5. / 4, 21. / 16, 4. / 3, 7. / 5, 35. / 24, 3. / 2, 63. / 40, 8. / 5,
       5. / 3, 7. / 4, 9. / 5, 28. / 15, 63. / 32)},
    {"harrison", T_RATIO, 16, 2,
     R(1, 16. / 15, 10. / 9, 8. / 7, 7. / 6, 6. / 5, 5. / 4, 4. / 3, 17. / 12, 3. / 2, 8. / 5, 5. / 3, 12. / 7, 7. / 4,
       9. / 5, 15. / 8)},
    {"sruti", T_RATIO, 22, 2,
     R(1, 256. / 243, 16. / 15, 10. / 9, 9. / 8, 32. / 27, 6. / 5, 5. / 4, 81. / 64, 4. / 3, 27. / 20, 45. / 32,
       729. / 512, 3. / 2, 128. / 81, 8. / 5, 5. / 3, 27. / 16, 16. / 9, 9. / 5, 15. / 8, 243. / 128)},
    {"perret", T_RATIO, 19, 2,
     R(1, 21. / 20, 35. / 32, 9. / 8, 7. / 6, 6. / 5, 5. / 4, 21. / 16, 4. / 3, 7. / 5, 35. / 24, 3. / 2, 63. / 40, 8. / 5,
       5. / 3, 7. / 4, 9. / 5, 15. / 8, 63. / 32)},
    {"michael_harrison", T_RATIO, 34, 2,
     R(1, 28. / 27, 135. / 128, 16. / 15, 243. / 224, 9. / 8, 8. / 7, 7. / 6, 32. / 27, 6. / 5, 135. / 112, 5. / 4,
       81. / 64, 9. / 7, 21. / 16, 4. / 3, 112. / 81, 45. / 32, 64. / 45, 81. / 56, 3. / 2, 32. / 21, 14. / 9, 128. / 81,
       8. / 5, 224. / 135, 5. / 3, 27. / 16, 12. / 7, 7. / 4, 16. / 9, 15. / 8, 243. / 128, 27. / 14)},
    {"harmonic", T_RATIO, 24, 2,
     R(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24)},
    {"bp", T_EQUAL, 13, 3, R(19.019550008653874 / 13)}, /* Bohlen-Pierce: 13 steps of the 3:1 */
    {"wcAlpha", T_EQUAL, 15, 0, R(0.78)},               /* rep 0: n steps (no octave) */
    {"wcBeta", T_EQUAL, 19, 0, R(0.638)},
    {"wcGamma", T_EQUAL, 34, 0, R(0.351)},
};
#undef R
#define NTUNINGS ((int)(sizeof(g_tunings) / sizeof(g_tunings[0])))

static int32_t g_steps[800];     /* every tuning's steps, semitones Q16 */
static int g_off[NTUNINGS + 1];  /* where each tuning's steps start */
static int32_t g_rep[NTUNINGS];  /* the interval of repetition, semitones Q16 */

int scales_count(void) { return NSCALES; }
int tunings_count(void) { return NTUNINGS; }
const char* scale_name(int s) { return (s >= 0 && s < NSCALES) ? g_scales[s].name : "?"; }
const char* tuning_name(int t) { return (t >= 0 && t < NTUNINGS) ? g_tunings[t].name : "?"; }
int scale_size(int s) {
    const struct scale* c = &g_scales[s < 0 ? 0 : s >= NSCALES ? NSCALES - 1 : s];
    return c->n ? c->n : c->div;
}
int tuning_size(int t) { return g_tunings[t < 0 ? 0 : t >= NTUNINGS ? NTUNINGS - 1 : t].n; }

static int32_t q16(double semis) { return (int32_t)(semis * 65536.0 + (semis >= 0 ? 0.5 : -0.5)); }
static double semis_of_ratio(double r) { return 12.0 * d_log(r) / 0.69314718055994530942; }

void scales_init(void) {
    int o = 0;
    for (int t = 0; t < NTUNINGS; ++t) {
        const struct tuning* T = &g_tunings[t];
        g_off[t] = o;
        for (int i = 0; i < T->n; ++i)
            g_steps[o++] = q16(T->kind == T_RATIO   ? semis_of_ratio(T->v[i])
                               : T->kind == T_SEMIS ? T->v[i]
                                                    : T->v[0] * i);
        g_rep[t] = T->rep > 0 ? q16(semis_of_ratio(T->rep)) : q16(T->v[0] * T->n);
    }
    g_off[NTUNINGS] = o;
}

int32_t scale_pitch_q16(int scale, int tuning, int root, int degree) {
    if (scale < 0) scale = 0;
    if (scale >= NSCALES) scale = NSCALES - 1;
    if (tuning < 0) tuning = 0;
    if (tuning >= NTUNINGS) tuning = NTUNINGS - 1;
    const struct scale* s = &g_scales[scale];
    const struct tuning* T = &g_tunings[tuning];
    const int use_t = s->n == 0 || T->n == s->div; /* the tuning's own steps */
    const int size = s->n ? s->n : (use_t ? T->n : s->div);
    int oct = degree / size, idx = degree % size;
    if (idx < 0) {
        idx += size;
        --oct;
    }
    const int step = s->n ? s->step[idx] : idx;
    int32_t semis, rep;
    if (use_t) {
        semis = g_steps[g_off[tuning] + step];
        rep = g_rep[tuning];
    } else { /* the equal division into the scale's steps */
        semis = (int32_t)(((int64_t)step * (12 << 16)) / s->div);
        rep = 12 << 16;
    }
    return ((60 + root) << 16) + semis + oct * rep;
}
