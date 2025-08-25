// If you want to test another Estimator, substitue EstimatorXXXX_ to your estimator

#include "EstimatorPortN.h"
#include "Cpp_Estimators/Estimator2001_Kalman.h"
#include "Cpp_Estimators/Estimator2002_UnscentedKalmanEstiamtor.h"

EstimatorPortN StateSpaceModel3_;

static inline double sgn(double v) { return (v > 0) - (v < 0); }
static inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void StateSpaceModel3_StateTransitionFunction(double *In_State, double *Out_State, struct EstimatorPortN *estimator) {

    double T = estimator->CurrentTimestamp - estimator->StateUpdateTimestamp;
    if (T <= 0) T = 0;

    double k     = estimator->Double_Par[11];
    double c_int = estimator->Double_Par[12];

    // for (int axis = 0; axis < 2; ++axis) {
    //     int i = axis * 3;
    //     double v = In_State[i+1];
    //     double kvT = k * fabs(v) * T;
    //     double denom = (1.0 + kvT);

    //     Out_State[i+0] = In_State[i+0] + c_int * sgn(v) * log(1.0 + kvT);
    //     Out_State[i+1] = v / denom;
    //     Out_State[i+2] = - k * fabs(v) * v;
    // }

    for (int axis = 0; axis < 3; ++axis) {
        int i = axis * 3;

        Out_State[i+0] = In_State[i+0] + In_State[i+1]*T + 1/2*In_State[i+2]*T*T;
        Out_State[i+1] = In_State[i+1] + In_State[i+2]*T;
        Out_State[i+2] = In_State[i+2];
    }

}

void StateSpaceModel3_ObservationFunction(double *In_State, double *Out_Observation, EstimatorPortN *estimator) {

    double dx = In_State[0] - estimator->Double_Par[0];
    double dy = In_State[3] - estimator->Double_Par[1];
    double dz = In_State[6] - estimator->Double_Par[2];

    double rho_xy = sqrt(dx*dx + dy*dy);
    double range  = sqrt(dx*dx + dy*dy + dz*dz);
    double angle_azimuth = atan2(dy, dx);
    double angle_elevation = atan2(dz, rho_xy);
    double angle_x = asin(clamp(In_State[2] / estimator->Double_Par[10], -1.0, 1.0));
    double angle_y = asin(clamp(In_State[5] / estimator->Double_Par[10], -1.0, 1.0));

    Out_Observation[0] = angle_azimuth;
    Out_Observation[1] = angle_elevation;
    Out_Observation[2] = range;
    Out_Observation[3] = angle_x * sin(angle_azimuth) - angle_y * cos(angle_azimuth); // roll
    Out_Observation[4] = angle_x * cos(angle_azimuth) + angle_y * sin(angle_azimuth);   // pitch

}

void StateSpaceModel3_PredictionFunction(double *In_State, double *Out_PredictedState, EstimatorPortN *estimator) {
	double T = estimator->PredictTime;
    if (T < 0) T = 0;

    for (int axis = 0; axis < 3; ++axis) {
        int i = axis * 3;

        Out_PredictedState[i+0] = In_State[i+0] + In_State[i+1]*T + 1/2*In_State[i+2]*T*T;
        Out_PredictedState[i+1] = In_State[i+1] + In_State[i+2]*T;
        Out_PredictedState[i+2] = In_State[i+2];
    }
}

EXPORT void StateSpaceModel3_EstimatorPort(double *In_Observation, double In_Observation_Timestamp, struct EstimatorPortN *estimator) {
	for (int i = 0; i < estimator->Nz; i++)
	{
		estimator->CurrentObservation[i] = In_Observation[i];
	}
    estimator->ObservationTimestamp = In_Observation_Timestamp;
    estimator->CurrentTimestamp = In_Observation_Timestamp;
    Estimator2002_Estimation(estimator);
    estimator->StateUpdateTimestamp = estimator->CurrentTimestamp;
}

EXPORT void StateSpaceModel3_EstimatorPortTermination(struct EstimatorPortN *estimator) {
    Estimator2002_Termination();

    free(estimator->PortName);
    estimator->PortName = NULL;
    free(estimator->PortIntroduction);
    estimator->PortIntroduction = NULL;
    free(estimator->EstimatedState);
    estimator->EstimatedState = NULL;
    free(estimator->PredictedState);
    estimator->PredictedState = NULL;
    free(estimator->CurrentObservation);
    estimator->CurrentObservation = NULL;
    free(estimator->PredictedObservation);
    estimator->PredictedObservation = NULL;
    free(estimator->Matrix_F);
    estimator->Matrix_F = NULL;
    free(estimator->Matrix_G);
    estimator->Matrix_G = NULL;
    free(estimator->Matrix_B);
    estimator->Matrix_B = NULL;
    free(estimator->Matrix_H);
    estimator->Matrix_H = NULL;
    free(estimator->Matrix_P);
    estimator->Matrix_P = NULL;
    free(estimator->Matrix_Q);
    estimator->Matrix_Q = NULL;
    free(estimator->Matrix_R);
    estimator->Matrix_R = NULL;
    free(estimator->Int_Par);
    estimator->Int_Par = NULL;
    free(estimator->Double_Par);
    estimator->Double_Par = NULL;
    printf("EstimatorPort terminated.\n");
}


EXPORT void StateSpaceModel3_Initialization(EstimatorPortN *estimator) 
{
    char *PortNameTemp = "Nonlinear Quadrotor Estimator";
    char *PortIntroductionTemp = "Nonlinear f(x), h(x) with drag model and EO angles";

    #define StateSpaceModel3_NX 9
    #define StateSpaceModel3_NZ 5
    #define StateSpaceModel3_PredictStep 10
    #define StateSpaceModel3_Interval 0.1
    #define StateSpaceModel3_PredictTime 1
    #define StateSpaceModel3_ObservationTimestamp 0

    double X0[StateSpaceModel3_NX] = {\
    0,0,0, 0,0,0, 0,0,0\
    };
    double P[StateSpaceModel3_NX*StateSpaceModel3_NX] = {\
    1,0,0,0,0,0,0,0,0,\
    0,1,0,0,0,0,0,0,0,\
    0,0,1,0,0,0,0,0,0,\
    0,0,0,1,0,0,0,0,0,\
    0,0,0,0,1,0,0,0,0,\
    0,0,0,0,0,1,0,0,0,\
    0,0,0,0,0,0,1,0,0,\
    0,0,0,0,0,0,0,1,0,\
    0,0,0,0,0,0,0,0,1,\
    };
    double Q[StateSpaceModel3_NX*StateSpaceModel3_NX] = {\
    0.1,0,0,0,0,0,0,0,0,\
    0,1,0,0,0,0,0,0,0,\
    0,0,100,0,0,0,0,0,0,\
    0,0,0,0.1,0,0,0,0,0,\
    0,0,0,0,1,0,0,0,0,\
    0,0,0,0,0,100,0,0,0,\
    0,0,0,0,0,0,0.1,0,0,\
    0,0,0,0,0,0,0,1,0,\
    0,0,0,0,0,0,0,0,100\
    };
    double R[StateSpaceModel3_NZ*StateSpaceModel3_NZ] = {\
    0.1,0,0,0,0,\
    0,0.1,0,0,0,\
    0,0,1,0,0,\
    0,0,0,1,0,\
    0,0,0,0,10
    };
    double Int_Par[100] = {0};
    double Double_Par[100] = {0};
    
    Double_Par[0]  = 0.0;   // x_apt
    Double_Par[1]  = 0.0;   // y_apt
    Double_Par[2]  = 0.0;   // z_apt
    Double_Par[10] = 9.8;   // g
    Double_Par[11] = 0.0123;// k
    Double_Par[12] = 81.3;  // c_int

    estimator->Nx = StateSpaceModel3_NX;
    estimator->Nz = StateSpaceModel3_NZ;
    estimator->PredictStep = StateSpaceModel3_PredictStep;
    estimator->Intervel = StateSpaceModel3_Interval;
    estimator->PredictTime = StateSpaceModel3_PredictTime;
    estimator->ObservationTimestamp = StateSpaceModel3_ObservationTimestamp;

    estimator->PortName = (char *)malloc(100);
    estimator->PortIntroduction = (char *)malloc(1000);
    estimator->EstimatedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->PredictedState = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->CurrentObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->PredictedObservation = (double *)malloc(estimator->Nz * sizeof(double));
    estimator->Matrix_F = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_G = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_B = (double *)malloc(estimator->Nx * sizeof(double));
    estimator->Matrix_H = (double *)malloc(estimator->Nz * estimator->Nx * sizeof(double));
    estimator->Matrix_P = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_Q = (double *)malloc(estimator->Nx * estimator->Nx * sizeof(double));
    estimator->Matrix_R = (double *)malloc(estimator->Nz * estimator->Nz * sizeof(double));
    estimator->Int_Par = (int *)malloc(100 * sizeof(int));
    estimator->Double_Par = (double *)malloc(100 * sizeof(double));
    if (estimator->PortName == NULL || estimator->PortIntroduction == NULL ||\
        estimator->EstimatedState == NULL || estimator->PredictedState == NULL ||\
        estimator->CurrentObservation == NULL || estimator->PredictedObservation == NULL ||\
        estimator->Matrix_F == NULL || estimator->Matrix_G == NULL ||\
        estimator->Matrix_B == NULL || estimator->Matrix_H == NULL ||\
        estimator->Matrix_P == NULL || estimator->Matrix_Q == NULL ||\
        estimator->Matrix_R == NULL || estimator->Int_Par == NULL ||\
        estimator->Double_Par == NULL) {
        perror("Memory allocation failed");
        exit(EXIT_FAILURE);
    }

    strcpy(estimator->PortName, PortNameTemp);
    strcpy(estimator->PortIntroduction, PortIntroductionTemp);
    memcpy(estimator->EstimatedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->PredictedState, X0, estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_P, P, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_Q, Q, estimator->Nx * estimator->Nx * sizeof(double));
    memcpy(estimator->Matrix_R, R, estimator->Nz * estimator->Nz * sizeof(double));
    memcpy(estimator->Int_Par, Int_Par, 100 * sizeof(int));
    memcpy(estimator->Double_Par, Double_Par, 100 * sizeof(double));

    // Initiate pointer
    estimator->StateTransitionEquation = StateSpaceModel3_StateTransitionFunction;
    estimator->ObservationEquation = StateSpaceModel3_ObservationFunction;
    estimator->PredictionEquation = StateSpaceModel3_PredictionFunction;

    printf("%s is initialized\n", estimator->PortName);

    Estimator2002_Init(estimator);
}