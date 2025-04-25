
#ifndef VECTLAB_H_
#define VECTLAB_H_
#endif

#include <math.h>

typedef unsigned int SIZE;






class VectorOperations
{
public:
    // this function copies a vector to another vector of the same size
    void copy(float (&destinationVector)[3], float (&sourceVector)[3])
    {
        for (SIZE c = 0; c < 3; c += 1)
        {
            destinationVector[c] = sourceVector[c];
        }
    }

    // this function clears a vector
    void clear(float (&vector1)[3])
    {
        for (SIZE c = 0; c < 3; c += 1)
        {
            vector1[c] = 0;
        }
    }

    // this function transforms a vector using a transformation matrix
    void transform(float (&result_vector)[3], float (&transformationMatrix)[3][3], float (&vector)[3])
    {
        float buffer[3];
        float val = 0.0;

        for (SIZE row = 0; row < 3; row += 1)
        {
            for (SIZE col = 0; col < 1; col += 1)
            {
                for (SIZE count = 0; count < 3; count += 1)
                {
                    val += transformationMatrix[row][count] * vector[count];
                }
                buffer[row] = val;
                val = 0.0;
            }
        }

        clear(result_vector);
        copy(result_vector, buffer);
    }

};

VectorOperations vectOp;
