#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "commons/magnetic.h"

#define maxord 12

#ifdef MAGNETIC_COFF_FILE
static float sp[13];
static float cp[13];
static float fn[13];
static float fm[13];

static float k[13][13];
static float c[13][13];
static float cd[13][13];
static float snorm2d[13][13];
#else
static const float ro_sp[13] = {
  0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f
};

static const float ro_cp[13] = {
  1.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f
};

static const float ro_fn[13] = {
  0.000000f, 2.000000f, 3.000000f, 4.000000f, 5.000000f, 6.000000f, 7.000000f, 8.000000f, 9.000000f, 10.000000f, 11.000000f, 12.000000f, 13.000000f
};

static const float ro_fm[13] = {
  0.000000f, 1.000000f, 2.000000f, 3.000000f, 4.000000f, 5.000000f, 6.000000f, 7.000000f, 8.000000f, 9.000000f, 10.000000f, 11.000000f, 12.000000f
};
///////////////////////////////////////////////////////

static const float ro_k[13][13] = {
  {0.000000f, -0.000000f, 0.333333f, 0.266667f, 0.257143f, 0.253968f, 0.252525f, 0.251748f, 0.251282f, 0.250980f, 0.250774f, 0.250627f, 0.250518f},
  {0.000000f, 0.000000f, 0.000000f, 0.200000f, 0.228571f, 0.238095f, 0.242424f, 0.244755f, 0.246154f, 0.247059f, 0.247678f, 0.248120f, 0.248447f},
  {0.000000f, 0.000000f, -1.000000f, 0.000000f, 0.142857f, 0.190476f, 0.212121f, 0.223776f, 0.230769f, 0.235294f, 0.238390f, 0.240602f, 0.242236f},
  {0.000000f, 0.000000f, 0.000000f, -0.333333f, 0.000000f, 0.111111f, 0.161616f, 0.188811f, 0.205128f, 0.215686f, 0.222910f, 0.228070f, 0.231884f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.200000f, 0.000000f, 0.090909f, 0.139860f, 0.169231f, 0.188235f, 0.201238f, 0.210526f, 0.217391f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.142857f, 0.000000f, 0.076923f, 0.123077f, 0.152941f, 0.173375f, 0.187970f, 0.198758f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.111111f, 0.000000f, 0.066667f, 0.109804f, 0.139319f, 0.160401f, 0.175983f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.090909f, 0.000000f, 0.058824f, 0.099071f, 0.127820f, 0.149068f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.076923f, 0.000000f, 0.052632f, 0.090226f, 0.118012f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.066667f, 0.000000f, 0.047619f, 0.082816f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.058824f, 0.000000f, 0.043478f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.052632f, 0.000000f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, -0.047619f},
};

static const float ro_c[13][13] = {
  {0.000000f, -29438.200000f, -3666.750000f, 3379.500000f, 3970.312500f, -1834.087500f, 1001.962500f, 2190.581250f, 1216.617188f, 522.285156f, -360.851562f, 1033.347656f, -1320.388672f},
  {4796.300000f, -1493.500000f, 5221.613570f, -7200.275099f, 4509.091716f, 3660.985920f, 1279.741545f, -2692.142907f, 596.578125f, 1121.150508f, -1484.045050f, -652.941026f, -89.702746f},
  {-4923.181215f, -553.217028f, 1454.056653f, 2369.491211f, 460.965414f, 1473.255346f, 1080.467993f, -205.621751f, -947.792009f, 325.950125f, 42.138384f, -940.810339f, 397.564930f},
  {-348.133730f, 477.345197f, -424.852004f, 460.348570f, -701.957762f, -664.987847f, -1286.200257f, 1068.971704f, -128.400677f, -265.545088f, 99.168204f, 688.732817f, 779.064975f},
  {1567.778207f, -738.014236f, 377.961167f, -244.038291f, 51.543845f, -348.752903f, -154.974883f, 185.233963f, -553.439746f, 33.825443f, -58.435425f, -191.611175f, -438.224049f},
  {476.812662f, 1510.144369f, -564.274897f, 35.496479f, 70.577012f, 5.402018f, 31.644668f, 56.187635f, 197.246797f, -444.720509f, 133.048108f, 95.054159f, 300.619217f},
  {-379.952807f, 490.170818f, 588.802751f, -366.155446f, 18.847192f, 41.577815f, -47.220038f, -7.265474f, 79.636638f, -1.739793f, -28.924060f, -65.882350f, 20.829891f},
  {-1925.999471f, -564.735795f, 122.870311f, 302.548806f, 21.610629f, -67.084541f, -1.877054f, 3.818833f, -40.861274f, 65.541667f, 44.095008f, 4.960435f, 70.232329f},
  {677.015625f, -1026.307323f, 550.880325f, -387.675184f, 240.255498f, 41.191365f, -23.062805f, 1.504096f, -1.316084f, -23.514197f, 19.638231f, 38.692065f, -23.410776f},
  {-2777.395578f, 1162.555445f, 979.197512f, -383.355021f, -232.467539f, 137.443652f, 7.533525f, -10.077513f, 5.176920f, -6.334114f, -4.778613f, -1.762985f, -12.771626f},
  {802.844044f, -84.276768f, 760.289566f, 514.231738f, -583.933361f, -24.792051f, -84.181378f, -23.729529f, -2.920263f, -5.223926f, -2.137061f, 1.088138f, 1.886494f},
  {-0.000000f, 859.000744f, -196.780805f, -263.465365f, 110.896519f, -18.823528f, -104.169141f, -34.140057f, -22.918805f, -5.440690f, -1.333953f, 2.029928f, -2.503335f},
  {-897.027462f, 238.538958f, 1168.597463f, -1071.214341f, 100.206406f, 145.809237f, -11.705388f, 17.558082f, 5.108650f, -8.489224f, -0.556297f, 0.454214f, -0.000000f},
};

static const float ro_cd[13][13] = {
  {0.000000f, 7.000000f, -16.500000f, 6.000000f, -3.500000f, -2.362500f, -11.550000f, -8.043750f, -5.027344f, -9.496094f, 0.000000f, -0.000000f, 0.000000f},
  {-30.200000f, 9.000000f, -10.738715f, -17.452614f, -4.980587f, 6.099949f, -9.451562f, -7.093921f, 13.406250f, -12.740347f, -0.000000f, 0.000000f, 0.000000f},
  {-51.268704f, -14.982239f, 0.259808f, 3.872983f, -25.435273f, -6.148170f, -1.494423f, -8.688243f, -11.216473f, -0.000000f, -21.069192f, -0.000000f, -0.000000f},
  {19.902104f, -1.549193f, -1.581139f, -8.696264f, 10.876580f, 0.470621f, 15.940514f, 18.430547f, 20.709787f, 33.193136f, 33.056068f, 0.000000f, 0.000000f},
  {-2.213594f, 22.696090f, 7.948270f, -2.588285f, -2.958040f, 2.662236f, -8.730979f, 1.234893f, -2.673622f, -22.550295f, -11.687085f, -0.000000f, -48.691561f},
  {2.033316f, 17.675990f, -0.000000f, 7.321149f, -0.420936f, 0.982185f, 0.000000f, -3.704679f, 5.932235f, 0.000000f, -14.783123f, -15.842360f, -0.000000f},
  {5.670937f, -22.416348f, -11.955386f, 2.182745f, 0.465363f, 0.873201f, 0.806032f, -2.179642f, 2.746091f, 5.219379f, -0.000000f, 0.000000f, 0.000000f},
  {21.281762f, 14.480405f, -16.382708f, -2.469786f, -6.791912f, 0.242182f, 0.129452f, 0.453082f, -0.250683f, 0.000000f, -2.004319f, -0.000000f, -0.000000f},
  {-26.812500f, 33.649420f, -4.141957f, 16.041732f, -2.966117f, -3.432614f, 1.253413f, 0.062671f, 0.250683f, -0.000000f, -1.636519f, -0.000000f, 0.000000f},
  {-38.221040f, 10.865004f, -33.193136f, 16.912722f, 3.369095f, -0.000000f, -0.753352f, 1.291989f, 0.121810f, -0.182715f, -0.265478f, -0.881492f, -0.000000f},
  {0.000000f, 21.069192f, -33.056068f, 11.687085f, -7.391562f, 4.132009f, -0.000000f, -0.818260f, 0.530957f, -0.000000f, -0.000000f, -0.000000f, -0.000000f},
  {0.000000f, 40.904797f, 0.000000f, 23.951397f, -0.000000f, -0.000000f, 4.960435f, -0.000000f, -0.881492f, -0.000000f, -0.057998f, -0.057998f, -0.000000f},
  {-0.000000f, 0.000000f, -64.922081f, 48.691561f, -0.000000f, 0.000000f, -0.000000f, 0.000000f, 0.000000f, -0.000000f, 0.000000f, -0.056777f, -0.056777f},
};

static const float ro_snorm2d[13][13] = {
  {1.000000f, 1.000000f, 1.500000f, 2.500000f, 4.375000f, 7.875000f, 14.437500f, 26.812500f, 50.273438f, 94.960938f, 180.425781f, 344.449219f, 660.194336f},
  {0.000000f, 1.000000f, 1.732051f, 3.061862f, 5.533986f, 10.166581f, 18.903125f, 35.469604f, 67.031250f, 127.403467f, 243.286074f, 466.386447f, 897.027462f},
  {0.000000f, 0.000000f, 0.866025f, 1.936492f, 3.913119f, 7.685213f, 14.944232f, 28.960810f, 56.082367f, 108.650042f, 210.691920f, 409.047973f, 795.129861f},
  {0.000000f, 0.000000f, 0.000000f, 0.790569f, 2.091650f, 4.706213f, 9.962822f, 20.478385f, 41.419573f, 82.982840f, 165.280340f, 327.968008f, 649.220813f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.739510f, 2.218530f, 5.456862f, 12.348931f, 26.736220f, 56.375738f, 116.870850f, 239.513968f, 486.915609f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.701561f, 2.326814f, 6.174465f, 14.830586f, 33.690948f, 73.915615f, 158.423599f, 334.021352f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.671693f, 2.421825f, 6.865227f, 17.397931f, 41.320085f, 94.117642f, 208.298910f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.647260f, 2.506827f, 7.533525f, 20.043185f, 49.604353f, 117.053882f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.626707f, 2.583978f, 8.182596f, 22.760038f, 58.526941f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.609049f, 2.654785f, 8.814925f, 25.543251f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.593628f, 2.720345f, 9.432471f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.579979f, 2.781484f},
  {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.567768f},
};
#endif

#ifdef MAGNETIC_COFF_FILE
static void initFromFile(const char *path);
void initFromFile(const char *path) {
  FILE *wmmdat;
  float epoch;
  int n, m, j, D1, D2;
  float gnm, hnm, dgnm, dhnm, flnmj;
  static char model[20], c_str[81], c_new[5];

  wmmdat = fopen(path, "r");
  if (wmmdat == NULL) {
    fprintf(stderr, "Error opening model file \n");
    exit(1);
  }

  /* INITIALIZE CONSTANTS */
  sp[0] = 0.0f;
  cp[0] = 1.0f;
  snorm2d[0][0] = 1.0f;

  /* READ WORLD MAGNETIC MODEL SPHERICAL HARMONIC COEFFICIENTS */
  c[0][0] = 0.0f;
  cd[0][0] = 0.0f;

  fgets(c_str, 80, wmmdat);
  if (sscanf(c_str,"%lf%s", &epoch, model) < 2) {
    fprintf(stderr, "Invalid header in model file WMM.COF\n");
    exit(1);
  }

  while (1) {
    if (fgets(c_str, 80, wmmdat) == NULL)
      break;

    /* CHECK FOR LAST LINE IN FILE */
    for (int i=0; i<4 && (c_str[i] != '\0'); i++) {
      c_new[i] = c_str[i];
      c_new[i+1] = '\0';
    }

    if (strcmp("9999", c_new) == 0)
      break;

    /* END OF FILE NOT ENCOUNTERED, GET VALUES */
    sscanf(c_str,"%d%d%lf%lf%lf%lf",&n,&m,&gnm,&hnm,&dgnm,&dhnm);

    if (n > maxord)
      break;

    if (m > n || m < 0.0) {
      fprintf(stderr, "Corrupt record in model file WMM.COF\n");
      exit(1);
    }

    if (m <= n) {
      c[m][n] = gnm;
      cd[m][n] = dgnm;
      if (m != 0) {
        c[n][m-1] = hnm;
        cd[n][m-1] = dhnm;
      }
    } //if (m <= n)
  } //while true

  /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */
  snorm2d[0][0] = 1.0;
  fm[0] = 0.0;
  for (n=1; n<=maxord; n++) {
    snorm2d[0][n] = snorm2d[0][n-1] * (float)(2*n-1)/(float)n;
    j = 2;
    for (m=0,D1=1,D2=(n-m+D1)/D1; D2>0; D2--,m+=D1) {
      k[m][n] = (float)(((n-1)*(n-1))-(m*m))/(float)((2*n-1)*(2*n-3));
      if (m > 0) {
        flnmj = (float)((n-m+1)*j)/(float)(n+m);
        snorm2d[m][n] = snorm2d[m-1][n]*sqrt(flnmj);
        j = 1;

        c[n][m-1] *= snorm2d[m][n];
        cd[n][m-1] *= snorm2d[m][n];
      }
      c[m][n] *= snorm2d[m][n];
      cd[m][n] *= snorm2d[m][n];
    }
    fn[n] = (float)(n+1);
    fm[n] = (float)n;
  }
  k[1][1] = 0.0f;

  fclose(wmmdat);
}
///////////////////////////////////////////////////////
#endif

void magnetic_calculate(float alt,
                        float lat,
                        float lon,
                        float year,
                        float *declination,
                        float *inclination,
                        float *totalIntensity,
                        float *gridVariation) {
// WGS84 major and semi-major axis
#define majAxis   6378.137f
#define semiMajAxis   6356.7523142f
#define earthRadius  6371.2f

#define a2  (majAxis*majAxis)
#define b2  (semiMajAxis*semiMajAxis)
#define c2  (a2-b2)
#define a4  (a2*a2)
#define b4  (b2*b2)
#define c4  (a4-b4)
#define pi ((float)M_PI)
#define dtr (pi/180.0f)

  //todo move to some section?
#define epoch 2015.0f

  int n, m, D3, D4;
  float pp[13];
  float tc[13][13];
  float dp[13][13];
  float sp[13];
  float cp[13];
  float snorm2d[13][13];
  float dt;
  float rlon, rlat;
  float srlon, srlat;
  float crlon, crlat;
  float srlat2, crlat2;
  float q, q1, q2, ct, st, r2, r;
  float d, ca, sa, aor, ar, br, bt;
  float bp, bpp, par, temp1, temp2;
  float parp, bx, by, bz, bh;

  pp[0] = 1.0f;
  dp[0][0] = 0.0f;

  dt = year - epoch;
  rlon = lon*dtr;
  rlat = lat*dtr;
  srlon = sinf(rlon);
  srlat = sinf(rlat);
  crlon = cosf(rlon);
  crlat = cosf(rlat);
  srlat2 = srlat*srlat;
  crlat2 = crlat*crlat;

  //copy from readonly to actual
  for (int i = 0; i < 13; ++i) {
    sp[i] = ro_sp[i];
    cp[i] = ro_cp[i];
    for (int j = 0; j < 13; ++j) {
      snorm2d[i][j] = ro_snorm2d[i][j];
    }
  }

  sp[1] = srlon;
  cp[1] = crlon;

  //CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS.
  q = sqrtf(a2 - c2*srlat2);
  q1 = alt*q;
  q2 = ((q1+a2)/(q1+b2))*((q1+a2)/(q1+b2));
  ct = srlat/sqrtf(q2*crlat2+srlat2);
  st = sqrtf(1.0f - (ct*ct));
  r2 = (alt*alt) + 2.0f*q1 + (a4 - c4*srlat2)/(q*q);
  r = sqrtf(r2);
  d = sqrtf(a2*crlat2 + b2*srlat2);
  ca = (alt + d)/r;
  sa = c2*crlat*srlat/(r*d);

  for (m=2; m<=maxord; m++) {
    sp[m] = sp[1]*cp[m-1] + cp[1]*sp[m-1];
    cp[m] = cp[1]*cp[m-1] - sp[1]*sp[m-1];
  }

  aor = earthRadius/r;
  ar = aor*aor;
  br = bt = bp = bpp = 0.0f;
  for (n=1; n<=maxord; n++) {
    ar = ar*aor;
    for (m=0, D3=1 , D4=(n+m+D3)/D3; D4>0; D4--, m+=D3) {

      //COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
      //AND DERIVATIVES VIA RECURSION RELATIONS
      do {
        if (n == m) {
          snorm2d[m][n] = st * snorm2d[m-1][n-1];
          dp[m][n] = st * dp[m-1][n-1] + ct * snorm2d[m-1][n-1];
          break;
        }

        if (n == 1 && m == 0) {
          snorm2d[m][n] = ct * snorm2d[m][n-1];
          dp[m][n] = ct*dp[m][n-1] - st*snorm2d[m][n-1];
          break;
        }

        if (n > 1 && n != m) {
          if (m > n-2) snorm2d[m][n-2] = 0.0;
          if (m > n-2) dp[m][n-2] = 0.0;
          snorm2d[m][n] = ct * snorm2d[m][n-1] - ro_k[m][n]*snorm2d[m][n-2];
          dp[m][n] = ct*dp[m][n-1] - st*snorm2d[m][n-1] - ro_k[m][n]*dp[m][n-2];
        }
      } while(0);

      //TIME ADJUST THE GAUSS COEFFICIENTS
      tc[m][n] = ro_c[m][n]+dt*ro_cd[m][n];
      if (m != 0)
        tc[n][m-1] = ro_c[n][m-1]+dt*ro_cd[n][m-1];

      //ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
      par = ar * snorm2d[m][n];
      if (m == 0) {
        temp1 = tc[m][n]*cp[m];
        temp2 = tc[m][n]*sp[m];
      } else {
        temp1 = tc[m][n]*cp[m]+tc[n][m-1]*sp[m];
        temp2 = tc[m][n]*sp[m]-tc[n][m-1]*cp[m];
      }

      bt = bt-ar*temp1*dp[m][n];
      bp += (ro_fm[m]*temp2*par);
      br += (ro_fn[n]*temp1*par);

      //SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
      if (st == 0.0f && m == 1) {
        if (n == 1)
          pp[n] = pp[n-1];
        else
          pp[n] = ct*pp[n-1] - ro_k[m][n]*pp[n-2];
        parp = ar*pp[n];
        bpp += ro_fm[m] * temp2 * parp;
      }

    } //for (m=0, D3=1 , D4=(n+m+D3)/D3; D4>0; D4--, m+=D3)
  } //for (n=1; n<=maxord; n++)

  if (st == 0.0f) bp = bpp;
  else bp /= st;

  //ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
  //GEODETIC COORDINATES
  bx = -bt*ca - br*sa;
  by = bp;
  bz = bt*sa - br*ca;

  //COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
  //TOTAL INTENSITY (TI)
  bh = sqrtf((bx*bx)+(by*by));
  if (totalIntensity)
    *totalIntensity = sqrtf((bh*bh)+(bz*bz));
  if (declination)
    *declination = atan2f(by,bx)/dtr;
  if (inclination)
    *inclination = atan2f(bz,bh)/dtr;

  //COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
  //GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
  //(I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)
  //OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
  if (gridVariation) {
    *gridVariation = -999.0f;
    if (fabsf(lat) < 55.0f)
      return;
    if (lat > 0.0f && lon >= 0.0f) *gridVariation = *declination-lon;
    if (lat > 0.0f && lon < 0.0f) *gridVariation = *declination+fabsf(lon);
    if (lat < 0.0f && lon >= 0.0f) *gridVariation = *declination+lon;
    if (lat < 0.0f && lon < 0.0f) *gridVariation = *declination-fabsf(lon);
    if (*gridVariation > +180.0f) *gridVariation -= 360.0f;
    if (*gridVariation < -180.0f) *gridVariation += 360.0f;
  }

}
///////////////////////////////////////////////////////
