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
    int gama;
    int alpha;
    float TsFc;

    if (f->opt.n_channels > MAX_NUMBER_IIR_CHANNELS)
    {
//         dg_printf("IIR channels exceeded max number: %d.  Consider increasing max number", MAX_NUMBER_IIR_CHANNELS);
//         exit(1);
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
    const unsigned int n_channels = f->opt.n_channels;
    const unsigned int input_size = f->opt.input_size;
    const int beta = f->opt.beta;
    const int alpha_x = f->opt.alpha_x;
    const int bit_shift = f->opt.bit_shift;
    const float iir_to_adc = f->opt.iir_to_adc;

    for (j=0; j<input_size; j+=n_channels)
    {
        for (i=0; i<n_channels; i++)
        {
            f->accum[i] = beta*f->accum[i] + alpha_x*((int)input[j+i]);
            f->accum[i] >>= bit_shift;
        }
    }

    for (i=0; i<n_channels; i++)
        output[i] = iir_to_adc*(float)f->accum[i];
}

void iir_filter_s16(iif_filter_t *f, short input[], float output[])
{
    unsigned int i,j;
    const unsigned int n_channels = f->opt.n_channels;
    const unsigned int input_size = f->opt.input_size;
    const int beta = f->opt.beta;
    const int alpha_x = f->opt.alpha_x;
    const int bit_shift = f->opt.bit_shift;
    const float iir_to_adc = f->opt.iir_to_adc;
    
    for (j=0; j<input_size; j+=n_channels)
    {
        for (i=0; i<n_channels; i++)
        {
            f->accum[i] = beta*f->accum[i] + alpha_x*((int)input[j+i]);
            f->accum[i] >>= bit_shift;
        }
    }
    
    for (i=0; i<n_channels; i++)
        output[i] = iir_to_adc*(float)f->accum[i];
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
    int i;
    float alpha;
    
    if (sampleCount == 0)
        alpha = 1.0f;
    else
        alpha = 1.0f / (float)sampleCount;
    
    // Find running average
    for (i=0; i<arraySize; i++)
        mean[i] = (1.0f-alpha)*mean[i] + (alpha)*input[i];
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
    int i;
    double alpha;
    
    if (sampleCount == 0)
        alpha = 1.0;
    else
        alpha = 1.0 / (double)sampleCount;
    
    // Find running average
    for (i=0; i<arraySize; i++)
        mean[i] = (1.0-alpha)*mean[i] + (alpha)*(double)input[i];
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


void tripleToSingleImu(imu_t *result, const imu3_t *imu3)
{
    // Triple IMU Averaging - optimized for Cortex M33
    int nPqr[3] = {0};
    int nAcc[3] = {0};
    imus_t mean = {};
    const uint32_t status = imu3->status;  // Cache status word

    for (int d=0; d<NUM_IMU_DEVICES; d++)
    {
        const imus_t *I = &imu3->I[d];
        const uint32_t gyrMask = (IMU3_STATUS_GYR_X_OK << (d*IMU3_STATUS_IMU_OK_BITSIZE));
        const uint32_t accMask = (IMU3_STATUS_ACC_X_OK << (d*IMU3_STATUS_IMU_OK_BITSIZE));
        
        for (int a=0; a<3; a++)
        {
            if (status & (gyrMask << a))
            {
                mean.pqr[a] += I->pqr[a];
                ++nPqr[a];
            }
            if (status & (accMask << a))
            {
                mean.acc[a] += I->acc[a];
                ++nAcc[a];
            }
        }
    }

    result->status = imu3->status & IMU3_STATUS_SATURATION_MASK;
    for (int a=0; a<3; a++)
    {
        STATIC_ASSERT(NUM_IMU_DEVICES <= 10);   // NUM_IMU_DEVICES > 10 will break inv_count_upto10 
        if (nPqr[a])
        {
            result->I.pqr[a] = mean.pqr[a] * inv_count_upto10(nPqr[a]);
            result->status |= (IMU_STATUS_GYR_X_OK << a);
        }
        else
        {
            result->I.pqr[a] = 0.0f;
        }
        if (nAcc[a])
        {
            result->I.acc[a] = mean.acc[a] * inv_count_upto10(nAcc[a]);
            result->status |= (IMU_STATUS_ACC_X_OK << a);
        }
        else
        {
            result->I.acc[a] = 0.0f;
        }
    }
    result->time = imu3->time;
}


int tripleToSingleImuExc(imu_t *result, const imu3_t *di, bool *exclude)
{
    imu_t imu = {};
    imu.time = di->time;
    imu.status = di->status;

    int cnt = 0;

    for (int idev = 0; idev < NUM_IMU_DEVICES; idev++)
    {
        if (!exclude[idev])
        {
            add_Vec3_Vec3(imu.I.pqr, imu.I.pqr, di->I[idev].pqr);
            add_Vec3_Vec3(imu.I.acc, imu.I.acc, di->I[idev].acc);
            cnt++;
        }
    }

    if (cnt > 0)
    {
        float div = 1.0f / (float)cnt;
        mul_Vec3_X(imu.I.pqr, imu.I.pqr, div);
        mul_Vec3_X(imu.I.acc, imu.I.acc, div);
    }

    *result = imu;
    return cnt;
}

void tripleToSingleImuAxis(imu_t* result, const imu3_t* di, bool exclude_gyro[NUM_IMU_DEVICES], bool exclude_acc[NUM_IMU_DEVICES], int iaxis)
{
    // Optimized for Cortex M33: precompute masks and reduce branches
    float w = 0.0f, a = 0.0f;
    int cnt_gyro = 0, cnt_acc = 0;
    const uint32_t status = di->status;
    
    // Precompute base masks shifted by axis (done once instead of per-device)
    const uint32_t gyrBaseMask = IMU3_STATUS_GYR_X_OK << iaxis;
    const uint32_t accBaseMask = IMU3_STATUS_ACC_X_OK << iaxis;

    for (int idev = 0; idev < NUM_IMU_DEVICES; idev++)
    {
        // Shift the base mask for each device
        const uint32_t gyrMask = gyrBaseMask << (idev * IMU3_STATUS_IMU_OK_BITSIZE);
        const uint32_t accMask = accBaseMask << (idev * IMU3_STATUS_IMU_OK_BITSIZE);

        // Combine checks to reduce branching
        if (!exclude_gyro[idev] && (status & gyrMask))
        {
            w += di->I[idev].pqr[iaxis];
            cnt_gyro++;
        }
        if (!exclude_acc[idev] && (status & accMask))
        {
            a += di->I[idev].acc[iaxis];
            cnt_acc++;
        }
    }
    
    // Update gyro status and value
    if (cnt_gyro > 0)
    { 
        w *= inv_count_upto10(cnt_gyro);
        result->status |= (IMU_STATUS_GYR_X_OK << iaxis);
    }
    else
    {   // No valid data
        result->status &= ~(IMU_STATUS_GYR_X_OK << iaxis);
    }
    
    // Update accelerometer status and value
    if (cnt_acc > 0)
    { 
        a *= inv_count_upto10(cnt_acc);
        result->status |= (IMU_STATUS_ACC_X_OK << iaxis);
    }
    else
    {   // No valid data
        result->status &= ~(IMU_STATUS_ACC_X_OK << iaxis);
    }

    result->I.pqr[iaxis] = w;
    result->I.acc[iaxis] = a;
    result->time = di->time;
    // result->status = di->status & IMU3_STATUS_SATURATION_MASK;
}


void singleToTripleImu(imu3_t *result, imu_t *imu)
{
    result->time = imu->time;
    result->status = imu->status & IMU_STATUS_SATURATION_MASK;
    for (int d=0; d<NUM_IMU_DEVICES; d++)
    {
        cpy_Vec3_Vec3(result->I[d].pqr, imu->I.pqr);
        cpy_Vec3_Vec3(result->I[d].acc, imu->I.acc);
        result->status |= (imu->status & IMU_STATUS_IMU_OK_MASK) << d*IMU_STATUS_IMU_OK_BITSIZE;
    }
}


int preintegratedImuToIMU(imu_t *imu, const pimu_t *pImu)
{
    if (pImu->dt == 0.0f)
    {
        return 0;
    }

    imu->time = pImu->time;
    imu->status = pImu->status;
    float divDt = 1.0f / pImu->dt;
    mul_Vec3_X(imu->I.pqr, pImu->theta, divDt);
    mul_Vec3_X(imu->I.acc, pImu->vel, divDt);
    return 1;
}


int imuToPreintegratedImu(pimu_t *pImu, const imu_t *imu, float dt)
{
    if (dt == 0.0f)
    {
        return 0;
    }

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

#define CON_SCUL_INT_STEPS  2

void integratePimu(pimu_t *output, imu_t *imu, imu_t *imuLast)
{
    output->time = imu->time;
    output->status |= imu->status;                                                      // Bitwise OR to preserve IMU status
    output->status &= (~IMU_STATUS_IMU_OK_MASK) | (imu->status&IMU_STATUS_IMU_OK_MASK); // Clear OK bits in PIMU status if not set in IMU status

    //  output->dt += deltaThetaDeltaVelRiemannSum(output, imu, imuLast);
    //  output->dt += deltaThetaDeltaVelTrapezoidal(output, imu, imuLast);

    // Numerical integration of coning and sculling integrals using Bortz's rotation vector formula
    output->dt += deltaThetaDeltaVelBortz(output, imu, imuLast, CON_SCUL_INT_STEPS);
            
    //  // Roscoe integral
    //  static ixVector3 alpha_last = { 0 };
    //  static ixVector3 veloc_last = { 0 };
    //  static ixVector3 delta_alpha_last = { 0 };
    //  static ixVector3 delta_veloc_last = { 0 };
    //  output->dt += integrateDeltaThetaVelRoscoe(output, imu, imu, alpha_last, veloc_last, delta_alpha_last, delta_veloc_last);
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


void integrateDeltaThetaVelBortz(ixVector3 theta, ixVector3 vel, imus_t *imu, imus_t *imuLast, int Nsteps, float dt)
{
    ixVector3 wb, ab, deltaW, deltaA, thxwb, thxthxwb, thxab, thxthxab;
    float dti, Kw, mag_theta2, mag_theta4;
    static const float Kw0 = 0.08333333333333333f;   // 1.0f / 12.0f;
    static const float Kw1 = 0.00138888888888889f;   // 1.0f / 720.0f
    static const float Kw2 = 3.306878306878307e-05f; // 1.0f / 30240.0f
    // static const float Kw3 = 8.267195767195768e-07f; // 1.0f / 1209600.0f

    // for jj = 1: Nint
    //     wb = W0 + (jj - 1) / Nint * (W1 - W0);
    //     ab = A0 + (jj - 1) / Nint * (A1 - A0);
    //     % Bortz's formula
    //     phi_dot = wb + 0.5 * cross(phi, wb) + 1 / 12 * cross(phi, cross(phi, wb));
    //     % Savage's formula (faster): approximates phi with gyro integral
    //     % phi_dot = Wi + 0.5 * cross(WintB, wb)
    //     phi = phi + phi_dot * dt / Nint;
    //     dv = dv + (ab + cross(phi, ab)) * dt / Nint;
    // end

    const float div = 1.0f / (float)Nsteps;
    sub_Vec3_Vec3(deltaW, imu->pqr, imuLast->pqr);
    sub_Vec3_Vec3(deltaA, imu->acc, imuLast->acc);
    mul_Vec3_X(deltaW, deltaW, div);
    mul_Vec3_X(deltaA, deltaA, div);
    cpy_Vec3_Vec3(wb, imuLast->pqr);
    cpy_Vec3_Vec3(ab, imuLast->acc);
    dti = dt * div;

    for (int jj = 0; jj < Nsteps; jj++) 
    {
        // Coning and sculling integrals
        cross_Vec3(thxwb, theta, wb);
        cross_Vec3(thxthxwb, theta, thxwb);
        cross_Vec3(thxab, theta, ab);
        cross_Vec3(thxthxab, theta, thxab);
        mag_theta2 = DOT_VEC3(theta);
        mag_theta4 = mag_theta2 * mag_theta2;
        Kw = Kw0 + mag_theta2 * Kw1 + mag_theta4 * Kw2; // + mag_theta4 * mag_theta2 * Kw3; <--- the last term is negligibly small
        const float half = 0.5f;
        for (int i = 0; i < 3; i++) {
            theta[i] += (wb[i] + half * thxwb[i] + Kw * thxthxwb[i]) * dti;
            vel[i] += (ab[i] + thxab[i] + half * thxthxab[i]) * dti;
        }
        // Advance wb, ab (minor step)
        add_Vec3_Vec3(wb, wb, deltaW);
        add_Vec3_Vec3(ab, ab, deltaA);
    }
}


/* Direct numerical integration of coning and sculling integrals using Bortz's formula */
float deltaThetaDeltaVelBortz(pimu_t *output, imu_t *imu, imu_t *imuLast, int Nsteps)
{
    float dt = (float)(imu->time - imuLast->time);

    // IMU
    integrateDeltaThetaVelBortz(output->theta, output->vel, &(imu->I), &(imuLast->I), Nsteps, dt);

    // Update history
    copyImu(imuLast, imu);

    return dt;
}


#if 0
float integrateDeltaThetaVelRoscoe(
    pimu_t *output, 
    imu_t *imu, 
    imu_t *imuLast,     
    ixVector3 alpha_last,
    ixVector3 veloc_last,
    ixVector3 delta_alpha_last,
    ixVector3 delta_veloc_last
)
{
    ixVector3 tmp3;
    float dt = (float)(imu->time - imuLast->time);
        
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
    
    // Update history
    copyImu(imuLast, imu);
    
    return dt;
}
#endif

void zeroPimu(pimu_t *pimu)
{
    pimu->time = 0.0;
    pimu->dt = 0.0f;
    pimu->status = 0;
    pimu->theta[2] = pimu->theta[1] = pimu->theta[0] = 0.0f;
    pimu->vel[2]   = pimu->vel[1]   = pimu->vel[0]   = 0.0f;
}
