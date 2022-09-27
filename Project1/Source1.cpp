
#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <ctype.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include "cv-helpers.hpp"   

using namespace std;
using namespace cv;
using namespace rs2;

struct Param_Kvadrata
{
	int i;
	int j;
	int nomer;
};

Mat Rectangle(Mat, int i, int j);
vector<Param_Kvadrata> Segmentation(Mat, int i, int j);
void Kvadrat(vector<Param_Kvadrata>, Mat);

int main(int argc, char* argv[]) try
{
	// —оздать конвейер - это API верхнего уровн€ дл€ потоковой передачи и обработки кадров.
	pipeline p;
	colorizer color_map;
	// Ќастроить и запустить конвейер
	p.start();
	colorizer colorize;

	auto gen_element = [](int erosion_size)
	{
		return getStructuringElement(MORPH_RECT, Size(erosion_size + 1, erosion_size + 1), Point(erosion_size, erosion_size));
	};
	const int erosion_size = 3;
	auto erode_less = gen_element(erosion_size);
	auto erode_more = gen_element(erosion_size * 4);
	auto create_mask_from_depth = [&](Mat& depth, int thresh, ThresholdTypes type)
	{
		threshold(depth, depth, thresh, 255, type);
		dilate(depth, depth, erode_less);
		erode(depth, depth, erode_more);
	};

	const auto window_name = "Display Image"; //!!! (c++ сам определ€ет тип данных)
	namedWindow(window_name, WINDOW_AUTOSIZE); // создание окна(им€,флаг)

	//WINDOW_AUTOSIZE - пользователь не может изменить размер окна, размер ограничен отображаемым изображением.
	for (int i = 0; i < 10; i++) p.wait_for_frames();
	while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
		// Ѕлокировать программу, пока не по€в€тс€ кадры
		frameset frames = p.wait_for_frames();
		frame depth1 = frames.get_depth_frame().apply_filter(color_map); //получить первый кадр глубины

		// ѕолучить кадр глубинного изображени€
		depth_frame depth = frames.get_depth_frame();
		colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);

		frame bw_depth = depth.apply_filter(colorize);
		// –азмер рамки запроса (ширина и высота)
		const int w = depth1.as<video_frame>().get_width();
		const int h = depth1.as<video_frame>().get_height();

		Mat image(Size(w, h), CV_8UC3, (void*)depth1.get_data(), Mat::AUTO_STEP);//w=640, h=480

		auto near_posle = frame_to_mat(bw_depth);
		cvtColor(near_posle, near_posle, COLOR_BGR2GRAY);
		create_mask_from_depth(near_posle, 240, THRESH_BINARY);


		auto near = frame_to_mat(bw_depth);
		cvtColor(near, near, COLOR_BGR2GRAY);
		create_mask_from_depth(near, 240, THRESH_BINARY);

		Mat imgCan;

		vector<Param_Kvadrata> v1(near.rows / 40 * near.cols / 40);
		imgCan = Rectangle(near, near.rows / 40, near.cols / 40);
		v1 = Segmentation(imgCan, imgCan.rows / 40, imgCan.cols / 40);
		Kvadrat(v1, imgCan);

		//int iter = 0;
		//int iter1 = 0;
		//float sumPix = 0;

		//// ¬ыделим пам€ть
		//float** masRast = new float* [width];
		//for (int i = 0; i < width; i++)
		//{
		//    masRast[i] = new float[height];
		//}
		////засунем в него рассто€ни€ на пиксель
		//for (int i = 0; i < width; i++)
		//{
		//    for (int j = 0; j < height; j++)
		//    {
		//        masRast[i][j] = depth.get_distance(i, j);
		//    }
		//}
		////суммируем все рассто€ни€, и считаем сколько "не черных" пикселей
		//for (int i = 0; i < width; i++)
		//{
		//    for (int j = 0; j < height; j++)
		//    {
		//        sumPix = sumPix + masRast[i][j];
		//        if (masRast[i][j] != 0)
		//        {
		//            iter++;
		//        }
		//        iter1++; //хз
		//    }
		//}

		//float sredRast = sumPix / iter;
		//std::cout << "The camera is facing an object " << sredRast << " meters away\r";



		//for (int i = 0, int ii = 0; i < width; i++, ii = ii + 3)
		//{
		//    for (int j = 0; j < height; j++)
		//    {
		//        if (masRast[i][j] < sredRast)
		//        {
		//            image[ii][j] = masRast[i][j];
		//        }
		//    }
		//}

		////освобождаем пам€ть из рабства
		//for (int i = 0; i < width; i++)
		//{
		//    delete masRast;
		//}

		//// Update the window with new data
		imshow("Display Ima1ge2", image);
		imshow("Display Ima1ge", near);
		//imshow(window_name, near_posle);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}


Mat Rectangle(Mat mat, int delenie_i, int delenie_j)
{
	uint8_t lol = 255;
	uint8_t kek = 0;
	int black = 0;
	int white = 0;
	for (int i = 0; i < mat.rows; i = i + delenie_i)
	{
		for (int j = 0; j < mat.cols; j = j + delenie_j)
		{
			for (int k = i; k < i + delenie_i; k++)
			{
				for (int p = j; p < j + delenie_j; p++)
				{
					if (mat.at<uint8_t>(k, p) == kek)
					{
						black++;
					}
					if (mat.at<uint8_t>(k, p) == lol)
					{
						white++;
					}
				}
			}
			if (black > white)
			{
				for (int k = i; k < i + delenie_i; k++)
				{
					for (int p = j; p < j + delenie_j; p++)
					{
						mat.at<uint8_t>(k, p) = kek;
					}
				}
			}
			else
			{
				for (int k = i; k < i + delenie_i; k++)
				{
					for (int p = j; p < j + delenie_j; p++)
					{
						mat.at<uint8_t>(k, p) = lol;
					}
				}
			}
			black = 0;
			white = 0;
		}
	}
	return mat;
}

vector<Param_Kvadrata> Segmentation(Mat mat, int delenie_i, int delenie_j)
{
	Param_Kvadrata exemp1; // в случае бага сделать массив
	uint8_t kek = 0;
	uint8_t lol = 255;
	int nomer_object = 0;
	int nomer_ob_pred = nomer_object - 1;
	bool sled_ob = false;
	bool est_sosed = false;
	bool perviy_ob = false;
	int pustota = 0;
	bool ob_net = false;
	vector<bool> vec2(mat.cols / delenie_j);
	vector<Param_Kvadrata> vec(0);
	for (int j = 0; j < mat.cols; j = j + delenie_j)
	{
		for (int i = 0; i < mat.rows; i = i + delenie_i)
		{
			// отрезает пустоту до первого предмета
			if (mat.at<uint8_t>(i, j) == lol && perviy_ob == false)
			{
				nomer_object++;
				perviy_ob = true;
			}
			// провер€ет наличие предмета по i
			if (i == 0 && perviy_ob == true)
			{
				for (int ll = 0; ll < mat.rows; ll = ll + delenie_i)
				{
					if (mat.at<uint8_t>(ll, j) == kek)
					{
						pustota++;
					}
				}
				if (pustota == mat.rows / 12)
				{
					ob_net = true;
				}
				else
				{
					ob_net = false;
				}
				vec2[j / 16] = ob_net;
			}
		}
		pustota = 0;
	}
	// итерируем i а j фиксируем
	for (int j = 0; j < mat.cols; j = j + delenie_j)
	{
		for (int i = 0; i < mat.rows; i = i + delenie_i)
		{
			if (perviy_ob == true && vec2[j / 16] == true && vec2[(j / 16) - 1] == false)
			{
				nomer_object++;
			}
			// заполнение вектора
			if (mat.at<uint8_t>(i, j) == lol)
			{
				exemp1.i = j;
				exemp1.j = i;
				exemp1.nomer = nomer_object;
				vec.push_back(exemp1);
			}
		}
	}
	return vec;
}

void Kvadrat(vector<Param_Kvadrata> vec, Mat mat)
{
	Point pt1, pt2;
	int left_verh, right_hiz;
	int max_i = vec[0].i;
	int max_j = vec[0].j;
	int min_i = vec[0].i;
	int min_j = vec[0].j;
	int tek_sost;
	int sled_sost;
	int konec = 0;
	bool konec_b = false;
	for (int i = 0; i < vec.size(); i++)
	{
		// ничего не писать
		if (vec[i].nomer > 0)
		{
			konec++;
			if (konec == vec.size())
			{
				konec_b = true;
			}

			tek_sost = vec[i].nomer;
			if (i + 1 < vec.size())
			{
				sled_sost = vec[i + 1].nomer;
			}
			else
			{
				sled_sost = vec[i].nomer;
			}


			if (vec[i].i > max_i)
			{
				max_i = vec[i].i;
			}
			if (vec[i].j < min_j)
			{
				min_j = vec[i].j;
			}
			if (vec[i].j > max_j)
			{
				max_j = vec[i].j;
			}
			if (vec[i].i < min_i)
			{
				min_i = vec[i].i;
			}
			if (tek_sost != sled_sost || konec_b == true)
			{
				pt1.x = min_i;
				pt1.y = min_j;
				pt2.x = max_i + 16;
				pt2.y = max_j + 12;
				rectangle(mat, pt1, pt2, Scalar(150, 150, 150), LINE_4, LINE_4);


				left_verh = 0;
				right_hiz = 0;
				if (i + 1 < vec.size())
				{
					max_i = vec[i + 1].i;
					max_j = vec[i + 1].j;
					min_i = vec[i + 1].i;
					min_j = vec[i + 1].j;
				}

			}
		}
		// ничего не писать
	}
}