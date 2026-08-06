/*
 *  filters.c
 *
 *  Created: 3/17/2011 8:27:17 AM
 *      Author: waltj
 */ 

// #include "misc/debug.h"
#include "filters.h"

//_____ M A C R O S ________________________________________________________

//_____ D E F I N I T I O N S ______________________________________________

//_____ G L O B A L S ______________________________________________________

//_____ L O C A L   P R O T O T Y P E S ____________________________________

#if 0
float integrateDeltaThetaVelRoscoe(
    pimu_t *output, 
    imu_t *imu, 
    imu_t *imuLast, 
    ixVector3 alpha_last, 
    ixVector3 veloc_last, 
    ixVector3 delta_alpha_last, 
    ixVector3 delta_veloc_last);
#endif

//_____ F U N C T I O N S __________________________________________________

void init_iir_filter(iif_filter_t *f)
{
    int gama, alpha;
    float TsFc;

    if (f->opt.n_channels > MAX_NUMBER_IIR_CHANNELS)
    {
        //dg_printf("IIR channels exceeded max number: %d.  Consider increasing max number", MAX_NUMBER_IIR_CHANNELS); // NOTE: dg_printf is undefined
        exit(1);
    }

    f->opt.bit_shift = (ACCUM_WORD_NBITS-1-f->opt.sig_word_nbits)/2;
    
    // Gama = adc_to_iir = alpha + beta
    gama                = 1<<f->opt.bit_shift;
    TsFc                = f->opt.Fc/f->opt.Fs;
    alpha               = (int)(TsFc/(1.0f + TsFc) * (float)gama);
    f->opt.beta         = gama - alpha;
    // alpha_x allows us to combine the adc_to_iir and alpha multiplies into one step
    f->opt.alpha_x      = alpha<<f->opt.bit_shift;
    f->opt.iir_to_adc   = 1.0f/(float)gama;
}


void iir_filter_u16(iif_filter_t *f, unsigned short input[], float output[])
{
    unsigned int i,j;

    for (j=0; j<f->opt.input_size; j+=f->opt.n_channels)
    {
        for (i=0; i<f->opt.n_channels; i++)
        {
            f->accum[i] = f->opt.beta*f->accum[i] + f->opt.alpha_x*((int)input[j+i]);
            f->accum[i] >>= f->opt.bit_shift;
        }
    }

    for (i=0; i<f->opt.n_channels; i++)
        output[i] = f->opt.iir_to_adc*(float)f->accum[i];
}

void iir_filter_s16(iif_filter_t *f, short input[], float output[])
{
    unsigned int i,j;    
    
    for (j=0; j<f->opt.input_size; j+=f->opt.n_channels)
    {
        for (i=0; i<f->opt.n_channels; i++)
        {
            f->accum[i] = f->opt.beta*f->accum[i] + f->opt.alpha_x*((int)input[j+i]);
            f->accum[i] >>= f->opt.bit_shift;
        }
    }
    
    for (i=0; i<f->opt.n_channels; i++)
        output[i] = f->opt.iir_to_adc*(float)f->accum[i];
}


/** 
 * \brief Running Average Filter
 *  A running average of the input array is collected in the mean array.  Filter
 *  is reset when sampleCount equals 0.
 *
 * \param input         Floating point value to be included in the average.
 * \param mean          Average of input
 * \param arraySize     Array length of mean and input arrays.
 * \param sampleCount   Sample number of input.  0 causes filter to be reset.
 */
void running_mean_filter(float input[], float mean[], int arraySize, int sampleCount)
{
    float alpha = 1.0f;
    
    if (sampleCount > 0)
        alpha = 1.0f / (float)sampleCount;
    
    // Find running average
    for (int i = 0; i < arraySize; i++)
        mean[i] = (1.0f - alpha) * mean[i] + alpha * input[i];
}


/** 
 * \brief Running Average Filter (double)
 *  A running average of the input array is collected in the mean array.  Filter
 *  is reset when sampleCount equals 0.
 *
 * \param input         Double (float 64) value to be included in the average.
 * \param mean          Average of input
 * \param arraySize     Array length of mean and input arrays.
 * \param sampleCount   Sample number of input.  0 causes filter to be reset.
 */
void running_mean_filter_f64(double mean[], float input[], int arraySize, int sampleCount)
{
    double alpha;
    
    if (sampleCount == 0)
        alpha = 1.0;
    else
        alpha = 1.0 / (double)sampleCount;
    
    // Find running average
    for (int i = 0; i < arraySize; i++)
        mean[i] = (1.0 - alpha) * mean[i] + alpha * (double)input[i];
}


/**
 * \brief Recursive Moving Average and Variance Filter
 * Recursive computation of moving average and variance given their previous
 * values, new element in the set, number of elements in the set (window size) and
 * assuming that one of the elements in the set is removed when new one is
 * added (i.e. fixed window size).
 * Reference: http://math.stackexchange.com/questions/1063962/how-can-i-recursively-approximate-a-moving-average-and-standard-deviation
 *
 * \param mean          Moving average of the set
 * \param var           Moving variance of the set
 * \param input         Floating point value added to the set
 * \param sampleCount   Number of samples in the sliding window
 */
void recursive_moving_mean_var_filter(float *mean, float *var, float input, int sampleCount)
{
    float dx, div;

    if (sampleCount <= 0) return;

    dx = input - *mean;

    // Expected moving average
    div = 1.0f / (float)sampleCount;
    *mean += dx * div;

    // Expected moving variance
    *var = ((float)(sampleCount * sampleCount - sampleCount - 1) * (*var) + (float)(sampleCount - 1) * dx * dx) * div * div;
}


void multiToSingleImu(imu_t *result, const imus_t *imus, const int numDevices)
{
    STATIC_ASSERT(MAX_IMU_DEVICES <= 10);   // NUM_IMU_DEVICES > 10 will break inv_count_upto10 

    // Multiple IMU Averaging - optimized for speed
    int ndev;
    float mean;
    uint32_t mask, baseS, base, axisMaskBase;

    result->status = imus->status & IMUS_STATUS_SATURATION_MASK;

    // Loop over gyros (isens = 0) and accelerometers (isens = 1)
    for (int isens = 0; isens < 2; isens++)
    {
        float *res = (isens == 0) ? result->I.pqr : result->I.acc;

        if (isens == 0) {
            baseS = IMUS_STATUS_GYR_X_OK;
            base  = IMU_STATUS_GYR_X_OK;
        }
        else {
            baseS = IMUS_STATUS_ACC_X_OK;
            base  = IMU_STATUS_ACC_X_OK;
        }

        for (int iaxis = 0; iaxis < 3; iaxis++)
        {
            axisMaskBase = baseS << iaxis;
            mean = 0.0f;
            ndev = 0;
            for (int idev = 0; idev < numDevices; idev++)
            {
                mask = axisMaskBase << (idev * IMUS_STATUS_IMU_OK_BITSIZE);

                if (imus->status & mask) {
                    if (isens == 0) mean += imus->I[idev].pqr[iaxis];
                    else            mean += imus->I[idev].acc[iaxis];
                    ndev++;
                }
            }

            if (ndev > 0) {
                mean *= inv_count_upto10(ndev);
                result->status |= (base << iaxis);
            }
            res[iaxis] = mean;
        }
    }
    result->time = imus->time;
}


void multiToSingleImuAxis(imu_t* result, const imus_t* di, const int numDevices, bool exclude_gyro[MAX_IMU_DEVICES], bool exclude_acc[MAX_IMU_DEVICES], int iaxis)
{
    STATIC_ASSERT(MAX_IMU_DEVICES <= 10);   // NUM_IMU_DEVICES > 10 will break inv_count_upto10 

    float mean;
    int ndev;
    const bool *excl;
    uint32_t mask, baseS, base, axisMaskBase;

    // Loop over gyros (isens = 0) and accelerometers (isens = 1)
    for (int isens = 0; isens < 2; isens++)
    {
        float *res = (isens == 0) ? result->I.pqr : result->I.acc;

        if (isens == 0) {
            baseS = IMUS_STATUS_GYR_X_OK;
            base  = IMU_STATUS_GYR_X_OK;
            excl  = exclude_gyro;
        }
        else {
            baseS = IMUS_STATUS_ACC_X_OK;
            base  = IMU_STATUS_ACC_X_OK;
            excl  = exclude_acc;
        }
        axisMaskBase = baseS << iaxis;
        ndev = 0;
        mean = 0.0f;
        for (int idev = 0; idev < numDevices; idev++)
        {
            mask = axisMaskBase << (idev * IMUS_STATUS_IMU_OK_BITSIZE);

            if (!excl[idev] != ((di->status & mask) != 0))
            {
                volatile int j=0;
                j++;
            }

            if (!excl[idev] && (di->status & mask)) {
                if (isens == 0) mean += di->I[idev].pqr[iaxis];
                else            mean += di->I[idev].acc[iaxis];
                ndev++;
            }
        }
        if (ndev > 0) { 
            mean *= inv_count_upto10(ndev);
            result->status |= (base << iaxis);
        }
        else {
            result->status &= ~(base << iaxis);  // No valid data
        }
        res[iaxis] = mean;
    }
}


void singleToMultiImu(imus_t *result, imu_t *imu, const int numDevices)
{
    result->time = imu->time;
    result->status = imu->status & IMU_STATUS_SATURATION_MASK;
    for (int d=0; d<numDevices; d++)
    {
        cpy_Vec3_Vec3(result->I[d].pqr, imu->I.pqr);
        cpy_Vec3_Vec3(result->I[d].acc, imu->I.acc);
        result->status |= (imu->status & IMU_STATUS_IMU_OK_MASK) << d*IMU_STATUS_IMU_OK_BITSIZE;
    }
}


int preintegratedImuToImuI(imui_t *imu, const pimu_t *pImu, float divDt)
{
    mul_Vec3_X(imu->pqr, pImu->theta, divDt);
    mul_Vec3_X(imu->acc, pImu->vel, divDt);
    return 1;
}


int preintegratedImuToImu(imu_t *imu, const pimu_t *pImu)
{
    if (pImu->dt == 0.0f) return 0;

    imu->time = pImu->time;
    imu->status = pImu->status;
    return preintegratedImuToImuI(&imu->I, pImu, 1.0f / pImu->dt);
}


int imuToPreintegratedImu(pimu_t *pImu, const imu_t *imu, float dt)
{
    if (dt == 0.0f) return 0;

    pImu->time = imu->time;
    pImu->dt = dt;
    pImu->status = imu->status;
    mul_Vec3_X(pImu->theta, imu->I.pqr, dt);
    mul_Vec3_X(pImu->vel, imu->I.acc, dt);
    return 1;
}

void copyImu(imu_t *dst, const imu_t *src)
{
    dst->time = src->time;
    dst->status = src->status;
    cpy_Vec3_Vec3(dst->I.pqr, src->I.pqr);
    cpy_Vec3_Vec3(dst->I.acc, src->I.acc);
}

/* Bortz's formula for rotation vector derivative given current rotation vector theta and body angular rate omega
*/
static void bortz(const ixVector3 theta, const ixVector3 omega, ixVector3 theta_dot)
{
    ixVector3 thxwb, thxthxwb;
    float Kw, mag_theta2, mag_theta4;
    const static float Kw0 = 0.08333333333333333f;   // 1.0f / 12.0f;
    const static float Kw1 = 0.00138888888888889f;   // 1.0f / 720.0f
    const static float Kw2 = 3.306878306878307e-05f; // 1.0f / 30240.0f

    cross_Vec3(thxwb, theta, omega);
    cross_Vec3(thxthxwb, theta, thxwb);
    mag_theta2 = DOT_VEC3(theta);
    mag_theta4 = mag_theta2 * mag_theta2;
    Kw = Kw0 + mag_theta2 * Kw1 + mag_theta4 * Kw2; // + mag_theta4 * mag_theta2 * Kw3; <--- the last term is negligibly small
    for (int i = 0; i < 3; i++) {
        theta_dot[i] = omega[i] + 0.5f * thxwb[i] + Kw * thxthxwb[i];
    }
}


static void integrateDeltaThetaVelBortz(ixVector3 theta, ixVector3 dvel, imui_t *imu, float dt)
{
    ixVector3 thxab, thxthxab, theta_dot, theta_dot1, theta_dot2, theta_dot3, theta_next;
    const float dt_div6 = dt * 0.16666667f;
    const float dt_div2 = dt * 0.5f;

    // Coning integral using RK4 integration with Bortz
    bortz(theta, imu->pqr, theta_dot);

    mul_Vec3_X(theta_next, theta_dot, dt_div2);
    add_Vec3_Vec3(theta_next, theta_next, theta);  // theta1 = theta + 0.5 * dt * theta_dot
    bortz(theta_next, imu->pqr, theta_dot1);

    mul_Vec3_X(theta_next, theta_dot1, dt_div2);
    add_Vec3_Vec3(theta_next, theta_next, theta);  // theta2 = theta1 + 0.5 * dt * theta_dot1
    bortz(theta_next, imu->pqr, theta_dot2);

    mul_Vec3_X(theta_next, theta_dot2, dt);
    add_Vec3_Vec3(theta_next, theta_next, theta);  // theta3 = theta2 + dt * theta_dot2
    bortz(theta_next, imu->pqr, theta_dot3);

    for (int i = 0; i < 3; i++) {
        theta[i] += (theta_dot[i] + 2.0f * theta_dot1[i] + 2.0f * theta_dot2[i] + theta_dot3[i]) * dt_div6;
    }

    // Sculling integral using coning integral result
    cross_Vec3(thxab, theta, imu->acc);
    cross_Vec3(thxthxab, theta, thxab);
    for (int i = 0; i < 3; i++) {
        dvel[i] += (imu->acc[i] + thxab[i] + 0.5f * thxthxab[i]) * dt;
    }
}


#if 0
static void integrateDeltaThetaVelRoscoe(
    pimu_t *output, 
    imu_t *imu, 
    imu_t *imuLast,     
    ixVector3 alpha_last,
    ixVector3 veloc_last,
    ixVector3 delta_alpha_last,
    ixVector3 delta_veloc_last,
    float dt;
)
{
    ixVector3 tmp3;
        
    // Roscoe (EQ-32) coning integral
    ixVector3 term1;
    ixVector3 term2;
    ixVector3 alpha;
    ixVector3 veloc;
    ixVector3 delta_alpha;
    ixVector3 delta_veloc;

    //__________________________________________________________________________________________________________________
    // Roscoe (EQ-32) coning integral:     [DELTA THETA = sum((1/2)*(alpha_last+(1/6)*delta_alpha_last) >< delta_alpha)]
    mul_Vec3_X(alpha, imu->I.pqr, dt);                                        //alpha                <-- [pqr] * [dt]
    sub_Vec3_Vec3(delta_alpha, alpha, alpha_last);                            //delta_alpha        <-- [alpha] - [alpha_last]
    div_Vec3_X(tmp3, delta_alpha_last, 6.0f);                                //tmp3                <-- [delta_alpha_last] * [(1/6)]
    add_Vec3_Vec3(tmp3, alpha_last, tmp3);                                    //tmp3                <-- [alpha_last] + [(1/6)*delta_alpha_last]
    div_Vec3_X(tmp3, tmp3, 2.0f);                                            //tmp3               <-- [(alpha_last+(1/6)*delta_alpha_last)] * [(1/2)]
    cross_Vec3(term1, tmp3, delta_alpha);                                    //term1                <-- [(1/2)*(alpha_last+(1/6)*delta_alpha_last)]    >< [delta_alpha] 
    add_Vec3_Vec3(output->theta, output->theta, term1);                        //theta                <-- sum[(1/2)*(alpha_last+(1/6)*delta_alpha_last)><delta_alpha]
    cpy_Vec3_Vec3(alpha_last, alpha);                                        //alpha_last        <-- alpha           {age alpha}
    cpy_Vec3_Vec3(delta_alpha_last, delta_alpha);                            //delta_alpha_last  <-- delta_alpha     {age delta_alpha}
    //_________________________________________________________________________________________________________________
    // Roscoe (EQ-33) sculling integral:   [DELTA VELOC = sum((1/2)*(alpha_last+(1/6)*delta_alpha_last) >< delta_veloc)
    //                                                 + sum((1/2)*(veloc_last+(1/6)*delta_veloc_last) >< delta_alpha)]      
    mul_Vec3_X(veloc, imu->I.acc, dt);                                        //veloc                <-- [acc] * [dt]
    sub_Vec3_Vec3(delta_veloc, veloc, veloc_last);                            //delta_veloc        <-- [veloc] - [veloc_last]
    cross_Vec3(term1, tmp3, delta_veloc);                                    //term1                <-- [(1/2)*(alpha_last+(1/6)*delta_alpha_last)] >< [delta_veloc]
    div_Vec3_X(tmp3, delta_veloc_last, 6.0f);                                //tmp3                <-- [delta_veloc_last] * [(1/6)]
    add_Vec3_Vec3(tmp3, veloc_last, tmp3);                                    //tmp3                <-- [veloc_last] + [(1/6)*delta_veloc_last]
    div_Vec3_X(tmp3, tmp3, 2.0f);                                            //tmp3                <-- [(veloc_last+(1/6)*delta_veloc_last)] * [(1/2)]
    cross_Vec3(term2, tmp3, delta_alpha);                                    //term2                <-- [(1/2)*(veloc_last+(1/6)*delta_veloc_last)] >< [delta_alpha]
    add_Vec3_Vec3(output->uvw, output->uvw, term1);                            //uvw                <-- sum[(1/2)*(alpha_last+(1/6)*delta_alpha_last)><delta_veloc ...
    add_Vec3_Vec3(output->uvw, output->uvw, term2);                            //...                ...   +[(1/2)*(veloc_last+(1/6)*delta_veloc_last)><delta_alpha]
    cpy_Vec3_Vec3(veloc_last, veloc);                                        //veloc_last        <-- veloc           {age veloc}
    cpy_Vec3_Vec3(delta_veloc_last, delta_veloc);                            //delta_veloc_last  <-- delta_veloc     {age delta_veloc}
}
#endif


void integratePimu(pimu_t *output, imu_t *imu, imu_t *imuLast)
{
    output->time = imu->time;
    output->status |= imu->status;                                                      // Bitwise OR to preserve IMU status
    output->status &= (~IMU_STATUS_IMU_OK_MASK) | (imu->status&IMU_STATUS_IMU_OK_MASK); // Clear OK bits in PIMU status if not set in IMU status

    float dti = (float)(imu->time - imuLast->time); // integration time

    // Numerical integration of coning and sculling integrals using Bortz's rotation vector formula
    integrateDeltaThetaVelBortz(output->theta, output->vel, &(imu->I), dti);

    output->dt += dti;

    // Update history
    copyImu(imuLast, imu);
            
    //  // Roscoe integral
    //  static ixVector3 alpha_last = { 0 };
    //  static ixVector3 veloc_last = { 0 };
    //  static ixVector3 delta_alpha_last = { 0 };
    //  static ixVector3 delta_veloc_last = { 0 };
    //  integrateDeltaThetaVelRoscoe(output, imu, imu, alpha_last, veloc_last, delta_alpha_last, delta_veloc_last, dti);
}

/** 
 * \brief Integrate IMUs (imus_t) into array of preintegrated IMUs (PIMUs).
 * \param pimuArray      Array of preintegrated IMU outputs to be updated.  Should be initialized to zero and maintained across calls to this function.
 * \param arraySize      Number of PIMUs to integrate (should be equal to number of IMUs).
 * \param imuLastArray   Array of previous IMU readings for each IMU.  Should be initialized to zero and maintained across calls to this function.
 */
void integrateImusIntoPimuArray(pimu_t pimuArray[MAX_IMU_DEVICES], const int arraySize, const imus_t *imusIn, imu_t imuLastArray[MAX_IMU_DEVICES])
{
    imu_t imu = {0};
    imu.time = imusIn->time;
    for (int i=0; i<arraySize; i++)
    {
        imu.I = imusIn->I[i];
        // Extract per-IMU status bits and preserve both OK and saturation flags
        const uint32_t perImuOk = IMU_STATUS_IMU_OK_MASK & (imusIn->status >> (i * IMUS_STATUS_IMU_OK_BITSIZE));
        imu.status = perImuOk | (imusIn->status & IMUS_STATUS_SATURATION_MASK);
        integratePimu(&pimuArray[i], &imu, &imuLastArray[i]);
    }
}

float deltaThetaDeltaVelRiemannSum(pimu_t *output, imu_t *imu, imu_t *imuLast)
{
    ixVector3 tmp3;
    float dt = (float)(imu->time - imuLast->time);
    
    // Use Riemann Sum integral

    // IMU - Delta Theta
    mul_Vec3_X(tmp3, imu->I.pqr, dt);
    add_Vec3_Vec3(output->theta, output->theta, tmp3);

    // IMU - Delta Velocity
    mul_Vec3_X(tmp3, imu->I.acc, dt);
    add_Vec3_Vec3(output->vel, output->vel, tmp3);

    // Update history
    copyImu(imuLast, imu);

    return dt;
}


float deltaThetaDeltaVelTrapezoidal(pimu_t *output, imu_t *imu, imu_t *imuLast)
{
    ixVector3 tmp3;
    float dt = (float)(imu->time - imuLast->time);
    
    // Use Trapezoidal integral

    // Delta Theta
    add_Vec3_Vec3(tmp3, imu->I.pqr, imuLast->I.pqr);
    mul_Vec3_X(tmp3, tmp3, dt*0.5f);
    add_Vec3_Vec3(output->theta, output->theta, tmp3);

    // Delta Velocity
    add_Vec3_Vec3(tmp3, imu->I.acc, imuLast->I.acc);
    mul_Vec3_X(tmp3, tmp3, dt*0.5f);
    add_Vec3_Vec3(output->vel, output->vel, tmp3);

    // Update history
    copyImu(imuLast, imu);

    return dt;
}


void zeroPimu(pimu_t *pimu)
{
    pimu->time = 0.0;
    pimu->dt = 0.0f;
    pimu->status = 0;
    pimu->theta[2] = pimu->theta[1] = pimu->theta[0] = 0.0f;
    pimu->vel[2]   = pimu->vel[1]   = pimu->vel[0]   = 0.0f;
}
