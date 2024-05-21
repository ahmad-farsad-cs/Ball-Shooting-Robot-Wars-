

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>
#include <windows.h>
#include "serial_com.h"
#include "timer.h"


#include "image_transfer2.h"
#include "vision.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

int activate();
int deactivate();
int initialize_label_image();
int label_objects2();
int check_all_labelled_objects();
int threshold2_background_normalization(image &a, image &a_normalized, image &bg, image &b, int tvalue);
int reset_label_number(image &label, image &final_label, int &original_label_number, int new_label_number);

int FindTarget(int nlabels, int nobstacles);
int Scape(int nlabels, int nobstacles);

image a, b, rgb;
image bg1, bg2; 
image rgb0; 
image a_normalized; 
image rgb_ave;
image label; 
image our_robot_label; 
image target_label; 
image obstacle_label; 

int nlabels; 
int nobstacles; 
int	tvalue = 120; 
double r_obstacle = 70;
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	i2byte nlabel;
	int camera;

	camera = 0;
	activate_vision(camera);

	printf("\npress any key to begin.");
	getch();

	activate();

	
	copy(rgb, bg2);

	initialize_label_image();

	
	acquire_image(rgb0);

	label_objects2();

	copy(a, rgb);
	view_image(rgb);
	printf("\n after labeling and thresholding");
	getch();

	// check all objects
	check_all_labelled_objects();

	// track
	//Scape(nlabels, nobstacles);
	FindTarget(nlabels, nobstacles);

	// deactivate the program
	deactivate();

	deactivate_vision();

	printf("\n\ndone.\n");
	getch();

	return 0; // no errors
}

int activate()

{
	


	a.type = GREY_IMAGE;
	a.width = IMAGE_WIDTH;
	a.height = IMAGE_HEIGHT;

	a_normalized.type = GREY_IMAGE;
	a_normalized.width = IMAGE_WIDTH;
	a_normalized.height = IMAGE_HEIGHT;

	b.type = GREY_IMAGE;
	b.width = IMAGE_WIDTH;
	b.height = IMAGE_HEIGHT;

	bg1.type = GREY_IMAGE;
	bg1.width = IMAGE_WIDTH;
	bg1.height = IMAGE_HEIGHT;

	bg2.type = GREY_IMAGE;
	bg2.width = IMAGE_WIDTH;
	bg2.height = IMAGE_HEIGHT;

	rgb.type = RGB_IMAGE;
	rgb.width = IMAGE_WIDTH;
	rgb.height = IMAGE_HEIGHT;


	rgb_ave.type = RGB_IMAGE;
	rgb_ave.width = IMAGE_WIDTH;
	rgb_ave.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;


	our_robot_label.type = LABEL_IMAGE;
	our_robot_label.width = IMAGE_WIDTH;
	our_robot_label.height = IMAGE_HEIGHT;

	target_label.type = LABEL_IMAGE;
	target_label.width = IMAGE_WIDTH;
	target_label.height = IMAGE_HEIGHT;

	obstacle_label.type = LABEL_IMAGE;
	obstacle_label.width = IMAGE_WIDTH;
	obstacle_label.height = IMAGE_HEIGHT;

	

	allocate_image(a);
	allocate_image(b);
	allocate_image(a_normalized);
	allocate_image(bg1);
	allocate_image(bg2);
	allocate_image(rgb);
	allocate_image(rgb_ave);
	allocate_image(rgb0);
	allocate_image(label);
	allocate_image(our_robot_label);
	allocate_image(target_label);
	allocate_image(obstacle_label);

	return 0; 
}

int deactivate()
{

	free_image(a);
	free_image(a_normalized);
	free_image(b);
	free_image(bg1);
	free_image(bg2);
	free_image(rgb);
	free_image(rgb_ave);
	free_image(rgb0);
	free_image(label);

	free_image(our_robot_label);
	free_image(target_label);
	free_image(obstacle_label);

	return 0; 
}

int initialize_label_image()
{
	i2byte *pl_target;
	i2byte *pl_robot;
	i2byte *pl_obstacle;

	i2byte *pl_target_2;
	i2byte *pl_robot_2;
	i2byte *pl_obstacle_2;

	int size;

	size = label.width*label.height;

	pl_target = (i2byte*)target_label.pdata;
	pl_robot = (i2byte*)our_robot_label.pdata;
	pl_obstacle = (i2byte*)obstacle_label.pdata;


	for (int i = 0; i < size; i++)
	{
		pl_target_2 = pl_target + i;
		pl_robot_2 = pl_robot + i;
		pl_obstacle_2 = pl_obstacle + i;

		*pl_target_2 = 0;
		*pl_robot_2 = 0;
		*pl_obstacle_2 = 0;

	}

	return 0;
}

int label_objects2()
{
	
	copy(rgb0, a);

	copy(rgb0, rgb);     

	scale(a, a);

	lowpass_filter(a, a);

	

	threshold2_background_normalization(a, a_normalized, bg2, b, tvalue);
	copy(b, a);   

	
	erode(a, b);

	dialate(b, a);

	
	label_image(a, label, nlabels);

	return 0; 
}

int check_all_labelled_objects()

{
	ibyte *prgb_ave;
	ibyte *p0;
	i2byte *pl;
	i2byte *pl_our_robot;
	i2byte *pl_obstacle;
	i2byte *pl_target;

	int sum_r, sum_g, sum_b, count, r_ave, g_ave, b_ave;
	int position, size;
	int R, G, B; 

	double ic = 0.0, jc = 0.0;

	nobstacles = 1;
	

	size = label.width*label.height;
	pl = (i2byte *)label.pdata;  
	pl_our_robot = (i2byte *)our_robot_label.pdata;
	pl_obstacle = (i2byte *)obstacle_label.pdata;
	pl_target = (i2byte *)target_label.pdata;

	p0 = rgb0.pdata;
	prgb_ave = rgb_ave.pdata;

	for (int n = 1; n <= nlabels; n++)
	{
		sum_r = 0;
		sum_g = 0;
		sum_b = 0;
		count = 0;
		r_ave = 0;
		g_ave = 0;
		b_ave = 0;

		
		for (int i = 0; i < size; i++)
		{
			if (pl[i] == n)
			{
				count++;
				sum_b += p0[3 * i];
				sum_g += p0[3 * i + 1];
				sum_r += p0[3 * i + 2];
			}
		}
		b_ave = sum_b / count;
		g_ave = sum_g / count;
		r_ave = sum_r / count;

		
		for (int i = 0; i < size; i++)
		{
			if (pl[i] == n)
			{
				prgb_ave[3 * i] = b_ave;
				prgb_ave[3 * i + 1] = g_ave;
				prgb_ave[3 * i + 2] = r_ave;
			}
			else if (pl[i] == 0) 
			{
				prgb_ave[3 * i] = 255;
				prgb_ave[3 * i + 1] = 255;
				prgb_ave[3 * i + 2] = 255;
			}
		}
	}


	for (int n = 1; n <= nlabels; n++)
	{
		position = 0;
		double ic = 200.0, jc = 300.0; 
		int ip, jp;
		
		centroid2(a, label, n, ic, jc);  

		ip = ic;
		jp = jc;

		
		position = a.width*jp + ip;

		B = prgb_ave[position * 3];
		G = prgb_ave[position * 3 + 1];
		R = prgb_ave[position * 3 + 2];

		int G_B = abs(G - B);
		int R_B = abs(R - B);
		int R_G = abs(R - G);


		
		bool green = (R>29 && R<51 && G<108 && G>73 && B>46 && B<78);
		bool darkblue = (R>23 && R<55 && G>48 && G<85 && B>90 && B<133); 
		bool purple = (R>58 && R<82 && G>55 && G<69 && B>72 && B<99); 
		bool lightblue = (R>75 && R<102 && G<125 && G>103 && B>118 && B<146); 
		bool red = (R<135 && R>90 && G>35 && G<70 && B>25 && B<60); 
		bool black = (R<80 && R>15 && G<46 && G>18 && B<49 && B>25);
		bool yellow = (R_G < 20 && B < 80 && R > 110); 
																	

		if (green)
		{

			reset_label_number(label, target_label, n, 1);
		}

		
		else if (darkblue)
		{
			 
			reset_label_number(label, our_robot_label, n, 1);
		}
		else if (red)
		{
			
			reset_label_number(label, our_robot_label, n, 2); 
		}
		else if (black)
		{
			
			reset_label_number(label, obstacle_label, n, 1);
		}

		else 
		{
			// create the rgb_ave
			for (int i = 0; i < size; i++)
			{
				if (pl[i] == n)
				{
					prgb_ave[3 * i] = 255;
					prgb_ave[3 * i + 1] = 255;
					prgb_ave[3 * i + 2] = 255;
				}
			}
		}
	}


	return 0;
}

int draw_point_RGB(image &rgb, int ip, int jp, int R, int G, int B)
{
	ibyte *pa;
	int i, j, w = 3, pixel;

	// initialize pointer
	pa = rgb.pdata;

	if (rgb.type != RGB_IMAGE) {
		printf("\nerror in draw_point_RGB: input type not valid!");
		return 1;
	}

	// limit out of range values
	if (ip < w) ip = w;
	if (ip > a.width - w - 1) ip = a.width - w - 1;
	if (jp < w) jp = w;
	if (jp > a.height - w - 1) jp = a.height - w - 1;

	for (i = -w; i <= w; i++) {
		for (j = -w; j <= w; j++) {
			pixel = a.width*(jp + j) + (ip + i);
			pa[3 * pixel] = B;
			pa[3 * pixel + 1] = G;
			pa[3 * pixel + 2] = R;
		}
	}

	return 0;
}

int reset_label_number(image &label, image &final_label, int &original_label_number, int new_label_number)
// original label number chang into new label number
{
	int size;
	i2byte *pl, *plfinal;
	size = label.width*label.height;
	pl = (i2byte *)label.pdata;  // remember!
	plfinal = (i2byte *)final_label.pdata;



	// set the new label number
	for (int i = 0; i < size; i++)
	{
		if (pl[i] == original_label_number)
		{
			plfinal[i] = new_label_number;
		}
	}

	return 0;
}

int threshold2_background_normalization(image &a, image &a_normalized, image &bg, image &b, int tvalue)

{
	i4byte size, i;
	ibyte *pa, *pa_normalized, *pbg, *pb;

	pa = a.pdata;
	pa_normalized = a_normalized.pdata;
	pbg = bg.pdata;
	pb = b.pdata;

	if (a.height != b.height || a.width != b.width) {
		printf("\nerror : a, b are not the same size!");
		return 1;
	}

	if (a.type != GREY_IMAGE || a_normalized.type != GREY_IMAGE || b.type != GREY_IMAGE) {
		printf("\nerror : input type not valid!");
		return 1;
	}

	size = (i4byte)a.width*a.height;

	// threshold opeartion
	for (i = 0; i < size; i++)
	{
		if (pbg[i] - pa[i] < tvalue)
		{
			pb[i] = 0;
			pa_normalized[i] = 0;
		}
		else
		{
			pa_normalized[i] = pbg[i] - pa[i];
			pb[i] = 255;
		}
	}
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// OUR CUSTOM FUNCTIONS ///////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum TurnCommand { Turn_left, Turn_Right };

TurnCommand getTurnCommand(double robotAngle, double targetAngle, double &def)
{
	TurnCommand command;

	if (targetAngle > robotAngle)
	{
		if (abs(targetAngle - robotAngle) < 180)
		{
			command = Turn_left;
			def = abs(targetAngle - robotAngle);
		}
		else
		{
			command = Turn_Right;
			def = 360 - abs(targetAngle - robotAngle);
		}
	}
	else
	{
		if (abs(targetAngle - robotAngle) < 180)
		{
			command = Turn_Right;
			def = abs(targetAngle - robotAngle);
		}
		else
		{
			command = Turn_left;
			def = 360 - abs(targetAngle - robotAngle);
		}
	}

	return command;
}
double lastangel = 0;
double chooseBestAngle(double angle1, double angle2, double x, double y)
{
	double cornery = 80, cornerx = 150;

	if (y < cornery)
	{
		if (angle1 > 180 && angle1 < 0)
			return angle1;
		else
			return angle2;
	}
	else if (y > 480 - cornery)
	{
		if (angle1 > 0 && angle1 < 180)
			return angle1;
		else
			return angle2;
	}
	else if (x < cornerx)
	{
		if (angle1 > 90 && angle1 < 270)
			return angle1;
		else
			return angle2;
	}
	else if (x > 640 - cornerx)
	{
		if (angle1 > 270 || angle1 < 90)
			return angle1;
		else
			return angle2;
	}
	else
	{
		if (abs(lastangel - angle1) < 25)
			return angle1;
		else
			return angle2;
	}
}

double distance(double alpha, double beta)
{
	double phi = (int)abs(beta - alpha) % 360;       
	double distance = phi > 180 ? 360 - phi : phi;
	return distance;
}

bool is_obstacle_between(double angle_robot, double angle_obstacle, double obstacle_distance, double obstacle_r, double robot_r)
{
	double def = abs(angle_obstacle - angle_robot);
	double mo = sin(def * 3.14 / 180) * 180 / 3.14  * obstacle_distance;
	if (mo < robot_r + obstacle_r + 10)
		return true;
	else
		return false;
}

double getHidingAngle(double angle_robot, double angle_target, double robot_x, double robot_y, double target_x, double target_y, double obstacle_x, double obstacle_y, double ex)
{
	if (angle_target > angle_robot)//
	{
		if (abs(angle_target - angle_robot) > 180) return ex;

		else if (abs(angle_target - angle_robot) < 180) return -ex;
	}
	if (angle_target < angle_robot)//
	{
		if (abs(angle_target - angle_robot) > 180) return -ex;

		else if (abs(angle_target - angle_robot) < 180) return ex;
	}
}

double getBestAngle(double robot_x, double robot_y, double robot_r, double target_x, double target_y, double obstacle_x, double obstacle_y, double angle)
{
 if (obstacle_y >= 480 - robot_r && robot_r < obstacle_x && obstacle_x < 640 - robot_r)
 if ((robot_x < obstacle_x && obstacle_x < target_x) || (robot_x<obstacle_x&&target_x<obstacle_x&&robot_y>obstacle_y&& obstacle_y>target_y) ||
 (robot_y<obstacle_y&&obstacle_y<target_y&&robot_x>obstacle_x&&target_x>obstacle_x))
 return angle;
 else
 return -angle;
 else if (obstacle_y <= robot_r && robot_r < obstacle_x && obstacle_x < 640 - robot_r)
 if ((robot_x < obstacle_x&&obstacle_x < target_x) || (robot_x<obstacle_x&&target_x<obstacle_x&&target_y>obstacle_y&&obstacle_y>robot_y) ||
 (robot_x > obstacle_x&&target_x > obstacle_x&&robot_y > obstacle_y&&obstacle_y > target_y))
 return -angle;
 else
 return angle;
 else if (obstacle_x > 640 - robot_r)
 if ((target_y > obstacle_y&&obstacle_y > robot_y) || (robot_y<obstacle_y&&target_y<obstacle_y&&robot_x>obstacle_x&&obstacle_x>target_x) ||
 (robot_y > target_y&& target_y > obstacle_y&&robot_x < obstacle_x&&obstacle_x < target_x))
 return -angle;
 else
 return angle;
 else if (obstacle_x <= robot_r&&robot_r < obstacle_y&& obstacle_y < 480 - robot_r)
 if ((target_y > obstacle_y&&obstacle_y > robot_y) || (robot_y < obstacle_y&& target_y < obstacle_y&&robot_x < obstacle_x&&obstacle_x < target_x) ||
 (robot_y > obstacle_y&&target_y > obstacle_y&&robot_x > obstacle_x&&obstacle_x > target_x))
 return angle;
 else
 return -angle;
 else
 return -10000;
 
	if (abs(lastangel - angle) < 25)
		return angle;
	else
		return -angle;
}

double checkEdges(double expected_angle, double x, double y, double robot_r)
{
	if (x > 620 - robot_r)
	{
		if (expected_angle < 270 && expected_angle >= 180)
			expected_angle = 280;
		else if (expected_angle > 90 && expected_angle < 180)
			expected_angle = 80;
		printf("\nRobot Near EDGE\n");
	}
	else if (x < robot_r)
	{
		if (expected_angle >= 270)
			expected_angle = 260;
		else if (expected_angle < 90)
			expected_angle = 100;
		printf("\nRobot Near EDGE\n");
	}
	else if (y > 450 - robot_r/2)
	{
		if (expected_angle > 180 && expected_angle <= 270)
			expected_angle = 170;
		else if (expected_angle > 270)
			expected_angle = 10;
		printf("\nRobot Near EDGE\n");

	}
	else if (y < robot_r/2)
	{
		if (expected_angle > 0 && expected_angle <= 90)
			expected_angle = 350;
		else if (expected_angle > 90 && expected_angle < 180)
			expected_angle = 190;
		printf("\nRobot Near EDGE\n");

	}

	return expected_angle;
}

void sendTurnCommand(HANDLE h, TurnCommand direction, int pwm)
{
	const int NMAX = 64;
	char buffer_out[NMAX];
	int n;

	n = 4;
	buffer_out[0] = (direction == Turn_left ? 'a' : 'd');
	buffer_out[1] = (char)(pwm / 10 + 48);
	buffer_out[2] = (char)(pwm % 10 + 48);;
	buffer_out[3] = '\r';
	serial_send(buffer_out, n, h);
}

void preShooting(HANDLE h)
{
	const int NMAX = 64;
	char buffer_out[NMAX];
	int n;

	n = 1;
	buffer_out[0] = 'p'; //set Servo to zero
	serial_send(buffer_out, n, h);
}

void shoot(HANDLE h)
{
	const int NMAX = 64;
	char buffer_out[NMAX];
	int n;

	n = 1;
	buffer_out[0] = 'k'; //set Servo to zero
	serial_send(buffer_out, n, h);
}

void goForward(HANDLE h, int pwm)
{
	const int NMAX = 64;
	char buffer_out[NMAX];
	int n;

	n = 4;
	buffer_out[0] = 'w';
	buffer_out[1] = (char)(pwm / 10 + 48);
	buffer_out[2] = (char)(pwm % 10 + 48);;
	buffer_out[3] = '\r';
	serial_send(buffer_out, n, h);
}

void stopMoving(HANDLE h)
{
	const int NMAX = 64;
	char buffer_out[NMAX];
	int n;

	n = 1;
	buffer_out[0] = 's'; //set Servo to zero
	serial_send(buffer_out, n, h);
}

int FindTarget(int nlabels, int nobstacles)
{
	//Initial Variables
	HANDLE h1;

	open_serial("COM5", h1);

	printf("\npress c key to continue");
	while (!KEY('C')) Sleep(1);
	Sleep(1000);

	double ic1 = 200.0, jc1 = 300.0; 
	double ic2 = 200.0, jc2 = 300.0; 
	double ioc1 = 200.0, joc1 = 300.0; 
	double itc1 = 200.0, jtc1 = 300.0; 

	int found = 0;
	int turning_l = 0;
	int turning_r = 0;

	while (1)
	{
		//Proccessing Image and Positioning and Controll Robot
		{
			acquire_image(rgb0);

			initialize_label_image();

			label_objects2();

			check_all_labelled_objects();

			///////////////////////////////////////////////////////////////////
			centroid(a, our_robot_label, 1, ic1, jc1);
			centroid(a, our_robot_label, 2, ic2, jc2);

			centroid(a, obstacle_label, 1, ioc1, joc1);

			centroid(a, target_label, 1, itc1, jtc1);
			///////////////////////////////////////////////////////////////////
			draw_point_RGB(rgb_ave, (int)ic1, (int)jc1, 0, 0, 0);
			draw_point_RGB(rgb_ave, (int)ic2, (int)jc2, 255, 255, 255);
			draw_point_RGB(rgb_ave, (int)itc1, (int)jtc1, 255, 215, 0);
			draw_point_RGB(rgb_ave, (int)ioc1, (int)joc1, 255, 0, 255);
			///////////////////////////////////////////////////////////////////
			printf("\ncentroid of robot marker 1 darkblue[%d]: ic = %lf , jc = %lf", 1, ic1, jc1);
			printf("\ncentroid of robot marker 2 red(2)[%d]: ic = %lf , jc = %lf", 2, ic2, jc2);
			printf("\ncentroid of obstacle 1 purple[%d]: ic = %lf , jc = %lf", 1, ioc1, joc1);
			printf("\ncentroid of target 1 green[%d]: ic = %lf , jc = %lf", 1, itc1, jtc1);
			///////////////////////////////////////////////////////////////////
			const double PI = 3.14159265358979323846;
			double angle_robot = atan2((jc1 - jc2), (ic1 - ic2)) * 180 / PI;
			if (angle_robot < 0)angle_robot += 360;
			printf("\nangle of robot: %lf ", angle_robot);

			double angle_target = atan2((jc1 - jtc1), (ic1 - itc1)) * 180 / PI;
			if (angle_target < 0)angle_target += 360;
			printf("\nangle of target: %lf ", angle_target);

			double angle_obstacle = atan2((jc1 - joc1), (ic1 - ioc1)) * 180 / PI;
			if (angle_obstacle < 0)angle_obstacle += 360;
			printf("\nangle of obstacle: %lf ", angle_obstacle);

			double obstacle_distance = sqrt(pow(ic1 - ioc1, 2) + pow(jc1 - joc1, 2));
			printf("\ndistance to obstacle: %lf ", obstacle_distance);

			double target_distance = sqrt(pow(ic1 - itc1, 2) + pow(jc1 - jtc1, 2));
			printf("\ndistance to target: %lf ", target_distance);

			double angle_obstacle_target = atan2((joc1 - jtc1), (ioc1 - itc1)) * 180 / PI;
			if (angle_obstacle_target < 0)angle_obstacle_target += 360;
			printf("\nangle_obstacle_target: %lf ", angle_obstacle_target);

			double angle_obstacle_robot = atan2((joc1 - jc1), (ioc1 - ic1)) * 180 / PI;
			if (angle_obstacle_robot < 0)angle_obstacle_robot += 360;
			printf("\nangle_obstacle_robot: %lf ", angle_obstacle_robot);

			bool obstacle_between = false;
			///////////////////////////////////////////////////////////////////
			printf("\n********************************************************************************\n\n");

			if ((distance(angle_target, angle_obstacle) < (asin(r_obstacle / obstacle_distance)*(180)) / 3.14 && obstacle_distance < target_distance))
			{
				//There is obstacle between Robot and Traget
				obstacle_between = true;
				printf("\nobstacle between robot and target , Robot Must move\n");
				////////////////////////////////////////////////////////////////
				///Finding Desire Angle
				double t = 50;
				double r_robot = 80;
				double expected_angle = getBestAngle(ic1, jc1, r_robot, itc1, jtc1, ioc1, joc1, t);
				lastangel = expected_angle;

				if (expected_angle == -10000)
				{
					printf("\n\t** GET HIDING ANGLE");
					expected_angle = -getHidingAngle(angle_obstacle_robot, angle_obstacle_target, ic1, jc1, itc1, jtc1, ioc1, joc1, 60);
				}

				if (obstacle_distance < 125)// && (abs(angle_obstacle - angle_target) > 30 || obstacle_distance < 100))
					if (expected_angle > 0)
						expected_angle += 20;
					else
						expected_angle -= 20;

				expected_angle = angle_target - expected_angle;

				if (expected_angle > 360)expected_angle -= 360;
				if (expected_angle < 0)expected_angle += 360;

			//	expected_angle = checkEdges(expected_angle, ic1, jc1, 100);

				printf("\n\t* Expected angel: %lf ", expected_angle);
				////////////////////////////////////////////////////////////////
				if (abs(angle_robot - expected_angle) > 10)
				{
					double def = 0;
					if (getTurnCommand(angle_robot, expected_angle, def) == Turn_left)
					{
						sendTurnCommand(h1, Turn_left, 70);
						printf("\t\tTurn Left\n");
					}
					else
					{
						sendTurnCommand(h1, Turn_Right, 70);
						printf("\t\tTurn Right\n");
					}
				}
				else
				{
					goForward(h1, 95);
					Sleep(500);

					printf("\t\tGo Straight on\n");
				}
				preShooting(h1);
				found = 0;
			}
			
			else
			{
				if (obstacle_between)
				{
					goForward(h1, 95);
					printf("\t\tGo Straight on\n");
					Sleep(4000);
					obstacle_between = false;
				}

				//There is No obstacle between Robot and Target
				printf("\nSafe Path Found\t");
				if (abs(angle_target - angle_robot) < 4)//baze charkhesh
				{
					found++;
					// Robot and Target have same angle (ready to shoot)
					stopMoving(h1);
					Sleep(50);
					if (found > 5)
						shoot(h1);
					Sleep(5);
					printf("\t* ShOOoooooOOt\n");
				}
				else
				{
					found = 0;
					double def;
					if (getTurnCommand(angle_robot, angle_target, def) == Turn_left)
					{
						if (def > 100)
							sendTurnCommand(h1, Turn_left, 75);
						if (def > 15)
							sendTurnCommand(h1, Turn_left, 65);
						else
						{
							sendTurnCommand(h1, Turn_left, 95);
							Sleep(35);
							stopMoving(h1);
							Sleep(1);
						}
						printf("\t* Turn Left");
					}
					else
					{
						if (def > 100)
							sendTurnCommand(h1, Turn_Right, 75);
						else if (def > 15)
							sendTurnCommand(h1, Turn_Right, 65);
						else
						{
							sendTurnCommand(h1, Turn_Right, 95);
							Sleep(35);
							stopMoving(h1);
							Sleep(1);
						}
						printf("\t* Turn Righift");
					}
					preShooting(h1);
				}
			}
		}
		///////////////////////////////////////////////////////////////////
		//Show Proccessed Image
		view_image(rgb_ave);
	}
	return 0;
}

int Scape(int nlabels, int nobstacles)
{
	HANDLE h1;

	open_serial("COM5", h1);

	printf("\npress c key to continue");
	while (!KEY('C')) Sleep(1);
	Sleep(1000);

	double ic1 = 200.0, jc1 = 300.0; 
	double ic2 = 200.0, jc2 = 300.0; 
	double ioc1 = 200.0, joc1 = 300.0; 
	double itc1 = 200.0, jtc1 = 300.0; 

	while (1)
	{
		acquire_image(rgb0);

		initialize_label_image();

		label_objects2();

		check_all_labelled_objects();

		///////////////////////////////////////////////////////////////////
		centroid(a, our_robot_label, 1, ic1, jc1);
		centroid(a, our_robot_label, 2, ic2, jc2);

		centroid(a, obstacle_label, 1, ioc1, joc1);

		centroid(a, target_label, 1, itc1, jtc1);
		///////////////////////////////////////////////////////////////////
		draw_point_RGB(rgb_ave, (int)ic1, (int)jc1, 0, 0, 0);
		draw_point_RGB(rgb_ave, (int)ic2, (int)jc2, 255, 255, 255);
		draw_point_RGB(rgb_ave, (int)itc1, (int)jtc1, 255, 215, 0);
		draw_point_RGB(rgb_ave, (int)ioc1, (int)joc1, 255, 0, 255);
		///////////////////////////////////////////////////////////////////
		printf("\ncentroid of robot marker 1 darkblue[%d]: ic = %lf , jc = %lf", 1, ic1, jc1);
		printf("\ncentroid of robot marker 2 red(2)[%d]: ic = %lf , jc = %lf", 2, ic2, jc2);
		printf("\ncentroid of obstacle 1 purple[%d]: ic = %lf , jc = %lf", 1, ioc1, joc1);
		printf("\ncentroid of target 1 green[%d]: ic = %lf , jc = %lf", 1, itc1, jtc1);
		///////////////////////////////////////////////////////////////////
		const double PI = 3.14159265358979323846;
		double angle_robot = atan2((jc1 - jc2), (ic1 - ic2)) * 180 / PI;
		if (angle_robot < 0)angle_robot += 360;
		printf("\nangle of robot: %lf ", angle_robot);

		double angle_obstacle_target = atan2((joc1 - jtc1), (ioc1 - itc1)) * 180 / PI;
		if (angle_obstacle_target < 0)angle_obstacle_target += 360;
		printf("\nangle_obstacle_target: %lf ", angle_obstacle_target);

		double angle_obstacle_robot = atan2((joc1 - jc1), (ioc1 - ic1)) * 180 / PI;
		if (angle_obstacle_robot < 0)angle_obstacle_robot += 360;
		printf("\nangle_obstacle_robot: %lf ", angle_obstacle_robot);

		double angle_robot_obstacle = atan2((jc1 - joc1), (ic1 - ioc1)) * 180 / PI;
		if (angle_robot_obstacle < 0)angle_robot_obstacle += 360;
		printf("\nangle_robot_obstacle: %lf ", angle_robot_obstacle);

		double obstacle_distance = sqrt(pow(ic1 - ioc1, 2) + pow(jc1 - joc1, 2));
		printf("\ndistance to obstacle: %lf ", obstacle_distance);

		double target_distance = sqrt(pow(ic1 - itc1, 2) + pow(jc1 - jtc1, 2));
		printf("\ndistance to target: %lf ", target_distance);

		printf("\n********************************************************************************\n\n");
		///////////////////////////////////////////////////////////////////

		double def = abs(angle_obstacle_robot - angle_obstacle_target);

		if (abs(def - 180) < 10)
		{
			stopMoving(h1);
			printf("\n* Robot is in Safe position\n");
		}
		else
		{
			double expected_angle = 0;
			expected_angle = getHidingAngle(angle_obstacle_robot, angle_obstacle_target, ic1, jc1, itc1, jtc1, ioc1, joc1, 60);
			lastangel = expected_angle;

			if (obstacle_distance<150)
				if (expected_angle > 0)
					expected_angle += 25;
				else
					expected_angle -= 25;

			expected_angle = angle_robot_obstacle - expected_angle;

			if (expected_angle < 0)expected_angle += 360;
			if (expected_angle > 360)expected_angle -= 360;
			printf("\n\t* Expected angel: %lf ", expected_angle);

			if (abs(angle_robot - expected_angle) > 10)
			{
				if (getTurnCommand(angle_robot, expected_angle, def) == Turn_left)
				{
					sendTurnCommand(h1, Turn_left, 70);
					printf("\t* Turn Left");
				}
				else
				{
					sendTurnCommand(h1, Turn_Right, 70);
					printf("\t* Turn Right");
				}
			}
			else
			{
				goForward(h1, 85);
				printf("\t\tGo Straight on\n");
			}
		}
		view_image(rgb_ave);
	}

	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////