/*
MIT LICENSE

Copyright (c) 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "convert_ins.h"
#include "ISPose.h"
#include "ISEarth.h"


void convertIns1ToIns2(ins_1_t *ins1, ins_2_t *result)
{
    result->week		= ins1->week;
    result->timeOfWeek	= ins1->timeOfWeek;
    result->insStatus	= ins1->insStatus;
    result->hdwStatus	= ins1->hdwStatus;
    euler2quat(ins1->theta, result->qn2b);
    memcpy(result->uvw, ins1->uvw, sizeof(ixVector3));
    memcpy(result->lla, ins1->lla, sizeof(ixVector3d));
}

void convertIns2ToIns1(ins_2_t *ins2, ins_1_t *result, double *refLla)
{
    result->week		= ins2->week;
    result->timeOfWeek	= ins2->timeOfWeek;
    result->insStatus	= ins2->insStatus;
    result->hdwStatus	= ins2->hdwStatus;
    quat2euler(ins2->qn2b, result->theta);
    memcpy(result->uvw, ins2->uvw, sizeof(ixVector3));
    memcpy(result->lla, ins2->lla, sizeof(ixVector3d));
    if (refLla)
    {   // Use ref LLA to solve for NED
        llaDeg2ned_d(refLla, ins2->lla, result->ned);
    }
    else
    {   // No ref LLA
        memset(result->ned, 0, sizeof(ixVector3));
    }
}

void convertIns3ToIns1(ins_3_t *ins3, ins_1_t *result, double *refLla)
{
    result->week		= ins3->week;
    result->timeOfWeek	= ins3->timeOfWeek;
    result->insStatus	= ins3->insStatus;
    result->hdwStatus	= ins3->hdwStatus;
    quat2euler(ins3->qn2b, result->theta);
    memcpy(result->uvw, ins3->uvw, sizeof(ixVector3));
    memcpy(result->lla, ins3->lla, sizeof(ixVector3d));
    if (refLla)
    {   // Use ref LLA to solve for NED
        llaDeg2ned_d(refLla, ins3->lla, result->ned);
    }
    else
    {   // No ref LLA
        memset(result->ned, 0, sizeof(ixVector3));
    }
}

void convertIns4ToIns1(ins_4_t *ins4, ins_1_t *result, double *refLla)
{
    ixVector3d llaRad;

    result->week		= ins4->week;
    result->timeOfWeek	= ins4->timeOfWeek;
    result->insStatus	= ins4->insStatus;
    result->hdwStatus	= ins4->hdwStatus;

    quatConjRot(result->uvw, ins4->qe2b, ins4->ve);
    ecef2lla(ins4->ecef, llaRad);
    qe2b2EulerNedLLA(result->theta, ins4->qe2b, llaRad);
    lla_Rad2Deg_d(result->lla, llaRad);
    if (refLla)
    {   // Use ref LLA to solve for NED
        llaDeg2ned_d(refLla, result->lla, result->ned);
    }
    else
    {   // No ref LLA
        memset(result->ned, 0, sizeof(ixVector3));
    }
}
