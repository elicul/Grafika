#include "main.h"

int thresh = 100;
int max_thresh = 255;
cv::RNG rng(12345);

cv::vector<cv::vector<cv::Point>> contours;
cv::vector<cv::Vec4i> hierarchy;

b2Vec2 gravity(0.0f, 9.81f);
b2World world(gravity);

float pixel = 20;
float space = 50;
const int ball_num = 20;

ball::ball()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(space / pixel, 50.0f / pixel);
	space += 10;
	body = world.CreateBody(&bodyDef);
	b2CircleShape dynamicBox;
	dynamicBox.m_radius = 20.0f / pixel;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.3f;
	fixtureDef.restitution = 0.8f;
	body->CreateFixture(&fixtureDef);
};

int main(int argc, const char** argv)
{
	init();
	ball Balls[ball_num];
	float32 timeStep = 1.0f / 20.0f;
	int32 velocityIterations = 8;
	int32 positionIterations = 3;
	
	b2Body* bodyy;
	cv::VideoCapture capture(0);
	cam_setup(capture);
		
	int key = 0;
	cv::Mat frame, foreground, image;
	cv::BackgroundSubtractorMOG2 mog(5000, 16, false);

	while (key != 'q') {
		capture >> frame;
		key = cv::waitKey(1);
		GaussianBlur(frame, frame, cv::Size(3, 3), 0, 0);
		image = frame.clone();
		mog(frame, foreground, -10);
		dilate(foreground, foreground, cv::Mat());
		imshow("Foreground ", foreground);
		cv::Mat threshold_output = foreground.clone();
		// Find contours
		findContours(threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		// Draw contours + hull results
		cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
		double max = 0;
		int num = -1;
		cv::Scalar color = CV_RGB(255, 0, 0);

		for (size_t i = 0; i< contours.size(); i++)
		{
			double temp = contourArea(contours[i]);
			if (temp > max && temp > 500)
			{
				max = temp;
				num = i;
			}
		}

		if (num != -1)
		{
			drawContours(image, contours, num, color, 3, 8, cv::vector<cv::Vec4i>(), 0, cv::Point());
			int t = contours[num].size();
			b2Vec2* vs = new b2Vec2[t];
			for (size_t i = 0; i< contours[num].size(); i++) {
				vs[i].Set(contours[num][i].x / pixel, contours[num][i].y / pixel);
			}
			b2ChainShape chain;
			chain.CreateLoop(vs, t);

			b2BodyDef bd;
			bodyy = world.CreateBody(&bd);
			bodyy->CreateFixture(&chain, 0.0f);

			delete vs;
		}

		world.Step(timeStep, velocityIterations, positionIterations);

		for (size_t i = 0; i<10; i++)
		{
			b2Vec2 position = Balls[i].body->GetPosition();
			circle(image, cv::Point(position.x * pixel, position.y * pixel), 20, CV_RGB(0, 255, 0), 3, 8, 0);
		}

		if (num != -1)
		{
			world.DestroyBody(bodyy);
		}

		imshow("Capture ", image);
	}
	cam_stop(capture);
	return 0;
}

void init()
{
	b2Body* floorBody;
	b2BodyDef floorDef;
	b2FixtureDef floorFixtureDef;
	b2PolygonShape floorBox;

	floorBox.SetAsBox(640.0f / 2.0f / pixel, 0.0f / pixel);
	floorFixtureDef.shape = &floorBox;
	floorFixtureDef.restitution = 0;
	floorFixtureDef.density = 0.0f;
	floorFixtureDef.friction = 0.3f;

	floorDef.position.Set(640.0f / 2.0f / pixel, 480.0f / pixel);
	floorBody = world.CreateBody(&floorDef);
	floorBody->CreateFixture(&floorFixtureDef);

	b2Body* leftBody;
	floorBox.SetAsBox(0.0f, 480.0f / 2.0f / pixel);
	floorDef.position.Set(0 / pixel, 480.0f / 2.0f / pixel);
	leftBody = world.CreateBody(&floorDef);
	leftBody->CreateFixture(&floorFixtureDef);

	b2Body* rightBody;
	floorDef.position.Set(640.0f / pixel, 480.0f / 2.0f / pixel);
	rightBody = world.CreateBody(&floorDef);
	rightBody->CreateFixture(&floorFixtureDef);
}

void cam_setup(cv::VideoCapture capture)
{
	if (!capture.isOpened()) {
		std::cout << "Cannot open the video file" << std::endl;
	}

	capture.set(CV_CAP_PROP_FPS, 30);
	capture.set(CV_CAP_PROP_EXPOSURE, -2);
	capture.set(CV_CAP_PROP_BRIGHTNESS, 0);
	capture.set(CV_CAP_PROP_CONTRAST, 35);
	capture.set(CV_CAP_PROP_SATURATION, 60);
	capture.set(CV_CAP_PROP_GAIN, 64);

	cv::namedWindow("Foreground ", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Capture ", CV_WINDOW_AUTOSIZE);
}
void cam_stop(cv::VideoCapture capture)
{
	if (capture.isOpened()) {
		capture.release();

		cv::destroyWindow("Capture ");
		cv::destroyWindow("Foreground ");
	}
}