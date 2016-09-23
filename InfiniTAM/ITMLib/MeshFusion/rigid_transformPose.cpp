
#include "MeshFusion.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>  
#include <cv.h>
#include <armadillo>

using namespace std;
using namespace cv;
using namespace arma;



cv::Mat MeshFusion::rigid_transformPose(cv::Mat A, cv::Mat B)
{
	/*cv::Mat A(3, 3, CV_32FC1);
	cv::Mat B(3, 3, CV_32FC1);
	for (int i = 0; i < A.rows; i++) {
		for (int j = 0; j < A.cols; j++) {
			A.ptr<float>(i)[j] = (float)(i + 5.0*j*0.01);
			B.ptr<float>(i)[j] = (float)(i + j + 2.0*i*0.01);
		}
	}*/
	/*cout << "A = \n" << A << endl;
	cout << "B = \n" << B << endl;*/

	int N = A.rows;

	cv::Mat centroid_A(1, 3, CV_32FC1); 
	cv::Mat centroid_B(1, 3, CV_32FC1);

	for (int i = 0; i < centroid_A.cols; i++) {
		centroid_A.at<float>(i) = mean(A.col(i))[0]; //取每一列的平均值
	} 
	centroid_A = centroid_A.t();
	cv::Mat centroid_A_2 = cv::Mat::ones(1, N, CV_32FC1);
	cv::Mat centroid_A_3 = centroid_A * centroid_A_2;
	centroid_A_3 = centroid_A_3.t(); //centroid_A_3 等於 python 的 tile(centroid_A, (N, 1))

	for (int i = 0; i < centroid_B.cols; i++) {
		centroid_B.at<float>(i) = mean(B.col(i))[0]; //取每一列的平均值
	} 
	centroid_B = centroid_B.t();
	cv::Mat centroid_B_2 = cv::Mat::ones(1, N, CV_32FC1);
	cv::Mat centroid_B_3 = centroid_B * centroid_B_2;
	centroid_B_3 = centroid_B_3.t();//centroid_B_3 等於 python 的 tile(centroid_B, (N, 1))

	cv::Mat AA = A - centroid_A_3;
	cv::Mat BB = B - centroid_B_3;
	cv::Mat H = AA.t() * BB;

	//將 cv::Mat H 轉換成 arma::mat
	cv::Mat Ht = H.t();
	arma::fmat arma_H(Ht.ptr<float>(), Ht.rows, Ht.cols);
	arma::mat HH = conv_to<mat>::from(arma_H);                   

	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	arma::svd(U, S, V, HH);
	Vt = V.t();
	arma::mat ret_R = Vt.t() * U.t();

	
	if (det(ret_R) < 0) { //如果是非奇異矩陣
		//printf("%s\n", "Reflection detected\n");
		Vt.row(2) *= -1;
		//Vt.row(1) *= -1;
		ret_R = Vt.t() * U.t();
	}

	//將centroid_A, centroid_B 轉換成 arma::mat
	arma::fmat farma_centroid_At(centroid_A.ptr<float>(), centroid_A.rows, centroid_A.cols);
	arma::mat arma_centroid_At = conv_to<mat>::from(farma_centroid_At);
	arma::fmat farma_centroid_Bt(centroid_B.ptr<float>(), centroid_B.rows, centroid_B.cols);
	arma::mat arma_centroid_Bt = conv_to<mat>::from(farma_centroid_Bt);

	arma::mat ret_t = -ret_R * arma_centroid_At + arma_centroid_Bt; // 因為一開始centroid_A和centroid_B有轉置了，所以這邊不需要再轉置


	//將 arma::mat ret_t 轉換成 cv::Mat
	arma::fmat TT = conv_to<fmat>::from(ret_t);
	cv::Mat cv_ret_t(TT.n_cols, TT.n_rows, CV_32FC1, TT.memptr());
	cv_ret_t = cv_ret_t.t();
	//cout << "cv_ret_t = \n" << cv_ret_t << endl;

	return cv_ret_t;
}

