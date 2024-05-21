
#include <stdlib.h>
#include <stdio.h>

#include "image_transfer2.h"
#include "vision.h"
#include "timer.h"

// input rgb image B-G-R-A
extern image rgb_in, rgb_copy;

// sample time of current incoming image
extern double t_sample;

int activate_process()
// process image initialization
// executed at program startup before image acquisition takes place
{
	return 0;
}


int process_image()
// this function is called continuously from the call back function
{
	return 0;
}


int deactivate_process()
// deactivation code
{
	return 0;
}



