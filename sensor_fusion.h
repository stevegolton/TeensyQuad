#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include <stdint.h>
#include <vector3f.h>
#include "kalman.h"

typedef struct
{
	vector3f_t stRotation;
	stKALMAN_Cxt_t stKalmanPitch;
	stKALMAN_Cxt_t stKalmanRoll;

} stSENSORFUSION_Cxt_t;

void SENSORFUSION_Setup( stSENSORFUSION_Cxt_t *pstCxt );
vector3f_t *SENSORFUSION_Update( stSENSORFUSION_Cxt_t *pstCxt,
								   vector3f_t *pstGyro,
								   vector3f_t *pstAccel,
								   float fTimestep_s );

#endif
