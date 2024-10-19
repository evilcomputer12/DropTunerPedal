#ifndef PitchShift_H_ /* Include guard */
#define PitchShift_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Function prototypes */
void PitchShift(float pitch_factor, uint32_t* inbuf, uint32_t* outbuf);

/*
// for Fs = 44.1k Hz
#define BUFF_SIZE 1000
#define BUFFER_DEPTH 1764 //40e-3s * fs
#define DELAY_DEPTH_UP 529 //12e-3s * fs
#define INITIAL_DELAY_UP 1323 //30e-3s * fs
#define INITIAL_DELAY_DOWN 4 //0.1e-3s * fs
*/

/* B
#define BUFF_SIZE 180 // 0.03s * fs
#define BUFFER_DEPTH 882 //0.1s * fs
#define DELAY_DEPTH_UP 264 //0.033s * fs
#define INITIAL_DELAY_UP 661 //0.083s * fs
#define INITIAL_DELAY_DOWN 4 //0.0005s * fs

*/

///*E*/
//// for Fs = 8k Hz
//#define BUFF_SIZE 256         // 0.03s * fs
//#define BUFFER_DEPTH 352      // 40e-3s * fs
//#define DELAY_DEPTH_UP 96     // 12e-3s * fs
//#define INITIAL_DELAY_UP 240  // 30e-3s * fs
//#define INITIAL_DELAY_DOWN 4  // 0.1e-3s * fs



#define BUFF_SIZE 2177
#define BUFFER_DEPTH 3840
#define DELAY_DEPTH_UP 1152
#define INITIAL_DELAY_UP 2880
#define INITIAL_DELAY_DOWN 4

#define DELAY_DEPTH_DOWN INITIAL_DELAY_UP - DELAY_DEPTH_UP + INITIAL_DELAY_DOWN
#define PI_HALF 1.5708

int WET_MIX = 1;            // wet=1 means 100% pitch shift
int DRY_MIX = 0;            // dry=1 means 100% original
int bufferA[BUFFER_DEPTH];  // Transmit PING buffer
int bufferB[BUFFER_DEPTH];  // Transmit PONG buffer
int change_direction = 1;   // change from pitch up to down and vice versa

// globals for PitchShift fnc
int Ga, Gb, crossfading, crossfadingAB, q;
int bufferABptr, Aout, Bout, delaya, delayb;
int pitch_shift_up = 0;
float pitch_shift_rate, bufferAptr_float, bufferBptr_float;
int channel_feedback = 0;

void PitchShift(float pitch_factor, uint32_t* inbuf, uint32_t* outbuf) {
    int m, i, bufferAptr2_float, bufferBptr2_float;

    // Set pitch_shift_rate based off the pitch_factor arg
    if (pitch_factor > 1) {  // pitch UP
        pitch_shift_rate = pitch_factor - 1;
        pitch_shift_up = 1;
    } else if (pitch_factor < 1) {  // pitch DOWN
        pitch_shift_rate = 1 - pitch_factor;
        pitch_shift_up = 0;
    } else {  // no change
        pitch_shift_rate = 0;
    }

    if (change_direction == 1) {
        crossfading = 0;    // 1-crossfading, 0-not crossfading
        crossfadingAB = 0;  // 1-crossfade from A to B, 0-crossfade from B to A
        q = 1;              // counter
        Ga = 1;             // channel A gain
        Gb = 0;             // channel B gain
        Aout = 0;           // channel A output
        Bout = 0;           // channel B output
        bufferABptr = 0;    // set buffer A&B ptr to beginning

        // Initialize Delay via chA&B ptrs according to type of pitch shift
        if (pitch_shift_up == 1) {
            bufferAptr_float = INITIAL_DELAY_UP;  // Initialize delay ~25ms
            bufferBptr_float = INITIAL_DELAY_UP;
            delaya = INITIAL_DELAY_UP;
            delayb = INITIAL_DELAY_UP;
        } else {
            bufferAptr_float = INITIAL_DELAY_DOWN;  // Initialize delay ~0ms
            bufferBptr_float = INITIAL_DELAY_DOWN;
            delaya = INITIAL_DELAY_DOWN;
            delayb = INITIAL_DELAY_DOWN;
        }

        // initialize ch buffers to 0 except first spot where input will be shifted in
        for (i = 0; i < BUFFER_DEPTH; i++) {
            bufferA[i] = 0;
            bufferB[i] = 0;
        }
        change_direction = 0;
    }

    // Process buffers (circular handling)
    for (m = 0; m < BUFF_SIZE; m++) {
        // Handle circular buffer for A
        if (((bufferAptr_float + 1) >= 0) && ((bufferAptr_float + 1) <= BUFFER_DEPTH - 1)) {
            bufferAptr2_float = bufferA[(int)bufferAptr_float + 1];
        } else {
            if ((bufferAptr_float + 1) < 0)
                bufferAptr2_float = (BUFFER_DEPTH - 1) + (bufferAptr_float + 1);
            if ((bufferAptr_float + 1) > BUFFER_DEPTH - 1)
                bufferAptr2_float = (bufferAptr_float + 1) - (BUFFER_DEPTH - 1);
        }

        // Handle circular buffer for B
        if (((bufferBptr_float + 1) >= 0) && ((bufferBptr_float + 1) <= BUFFER_DEPTH - 1)) {
            bufferBptr2_float = bufferB[(int)bufferBptr_float + 1];
        } else {
            if ((bufferBptr_float + 1) < 0)
                bufferBptr2_float = (BUFFER_DEPTH - 1) + (bufferBptr_float + 1);
            if ((bufferBptr_float + 1) > BUFFER_DEPTH - 1)
                bufferBptr2_float = (bufferBptr_float + 1) - (BUFFER_DEPTH - 1);
        }

        // Update buffer with incoming data (now treating as uint32_t)
        bufferA[bufferABptr] = inbuf[m] + channel_feedback * bufferAptr2_float;
        bufferB[bufferABptr] = inbuf[m] + channel_feedback * bufferBptr2_float;

        // Compute channel outputs & system output
        Aout = Ga * WET_MIX * bufferA[(int)bufferAptr_float] + DRY_MIX * inbuf[m];
        Bout = Gb * WET_MIX * bufferB[(int)bufferBptr_float] + DRY_MIX * inbuf[m];
        outbuf[m] = (uint32_t)(Aout + Bout);  // Ensure output remains uint32_t

        // Calculate delays
        if (bufferABptr > (int)bufferAptr_float)
            delaya = BUFFER_DEPTH - (bufferABptr - (int)bufferAptr_float);
        else
            delaya = (int)bufferAptr_float - bufferABptr;

        if (bufferABptr > (int)bufferBptr_float)
            delayb = BUFFER_DEPTH - (bufferABptr - (int)bufferBptr_float);
        else
            delayb = (int)bufferBptr_float - bufferABptr;

        // Crossfade if necessary
        if ((pitch_shift_up == 1 && (delaya <= DELAY_DEPTH_UP || delayb <= DELAY_DEPTH_UP) && crossfading != 1) ||
            (pitch_shift_up == 0 && (delaya >= DELAY_DEPTH_DOWN || delayb >= DELAY_DEPTH_DOWN) && crossfading != 1)) {
            crossfading = 1;
            crossfadingAB = !crossfadingAB;
            q = 1;  // Controls fade between channels
        }

        // Handle crossfading
        if (crossfading == 1) {
            if (crossfadingAB) {
                Ga = cos((pitch_shift_rate / DELAY_DEPTH_UP) * PI_HALF * q);
                Gb = 1 - Ga;
                q++;
            } else {
                Gb = cos((pitch_shift_rate / DELAY_DEPTH_UP) * PI_HALF * q);
                Ga = 1 - Gb;
                q++;
            }

            // Stop crossfading when done
            if (q >= (int)((DELAY_DEPTH_UP) / pitch_shift_rate) && Ga < Gb) {
                Ga = 0;
                Gb = 1;
                q = 1;
                crossfading = 0;
                if (pitch_shift_up)
                    bufferAptr_float = INITIAL_DELAY_UP;
                else
                    bufferAptr_float = INITIAL_DELAY_DOWN;
            } else if (q >= (int)((DELAY_DEPTH_UP) / pitch_shift_rate) && Ga > Gb) {
                Gb = 0;
                Ga = 1;
                q = 1;
                crossfading = 0;
                if (pitch_shift_up)
                    bufferBptr_float = INITIAL_DELAY_UP;
                else
                    bufferBptr_float = INITIAL_DELAY_DOWN;
            }
        }

        // Adjust channel delays for pitch shift
        if (pitch_shift_up) {
            if (Ga != 0) bufferAptr_float -= pitch_shift_rate;
            if (Gb != 0) bufferBptr_float -= pitch_shift_rate;
        } else {
            if (Ga != 0) bufferAptr_float += pitch_shift_rate;
            if (Gb != 0) bufferBptr_float += pitch_shift_rate;
        }

        // Decrement pointers
        bufferABptr--;
        bufferAptr_float--;
        bufferBptr_float--;

        // Circular buffer bounds check
        if (bufferABptr < 0) bufferABptr = BUFFER_DEPTH - 1;
        if (bufferAptr_float < 0) bufferAptr_float = BUFFER_DEPTH - 1 + bufferAptr_float;
        if (bufferAptr_float > BUFFER_DEPTH - 1) bufferAptr_float -= BUFFER_DEPTH - 1;
        if (bufferBptr_float < 0) bufferBptr_float = BUFFER_DEPTH - 1 + bufferBptr_float;
        if (bufferBptr_float > BUFFER_DEPTH - 1) bufferBptr_float -= BUFFER_DEPTH - 1;
    }
}


#endif  // PitchShift_H_
