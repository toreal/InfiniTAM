<<<<<<< HEAD

#include "MeshFusion.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>  
#include <armadillo>

using namespace std;
using namespace arma;


arma::mat ret_R;
arma::mat ret_t;
void rigid_transform_3D(arma::mat A, arma::mat B);


void MeshFusion::rigid_transformPose()
{

	arma::mat R = arma::randu<arma::mat>(3, 3) * 1; //產生3*3隨機矩陣，每個值的範圍為0~1
	arma::mat t = arma::randu<arma::mat>(3, 1) * 1; //經測試發現產生的隨機值好像是固定的
	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	svd(U, S, V, R);//對矩陣 R 進行奇異值分解 
	Vt = V.t();
	/*R.print("R:");
	U.print("U=\n");
	S.print("S=\n");
	V.print("V=\n");
	Vt.print("Vt=\n");*/
	R = U*Vt;
	//R.print("Rotation:");

	if (det(R) < 0) {
		Vt.row(2) *= -1;
		R = U*Vt;
	}


	int n = 10;

	arma::mat t2;
	t2.ones(1, n); //產生nx1全1矩陣
	arma::mat t3 = t * t2; // t3 等於 python 的 tile(t, (1, n))
	//t3.print("t3=\n");

	arma::mat A = arma::randu<arma::mat>(n, 3) * 1;
	//A.print("A=\n");
	arma::mat B = R * A.t() + t3; // t3 等於 python 的 tile(t, (1, n))
	//t3.print("t3:");
	B = B.t();

	rigid_transform_3D(A, B);


	arma::mat ret_t2;
	ret_t2.ones(1, n);//產生1xn全1矩陣
	//ret_t2.print("ret_t2:");
	arma::mat ret_t3 = ret_t * ret_t2;
	arma::mat A2 = (ret_R*A.t()) + ret_t3;//ret_t3 等於 tile(ret_t, (1, n))

	A2 = A2.t();

	arma::mat err = A2 - B;
	
	err = err % err; //點乘積 ,同 python 的 numpy.multiply()
	arma::mat c1 = sum(err, 1); //err各行的總和
	
	arma::mat r1 = sum(err, 0); //err各列的總和
	
	arma::mat r2 = sum(c1, 0);
	
	arma::mat c2 = sum(r1, 1);
	
	arma::mat err_sum = r2 + c2;
	//err_sum.print("err_sum:");
	A.print("Points A:");
	B.print("Points B:");
	R.print("Rotation:");
	t.print("Translation:");
	arma::mat rmse = sqrt(err_sum / n);
	rmse.print("rmse:");
	printf("If RMSE is near zero, the function is correct!\n");
	//system("pause");
}

void rigid_transform_3D(arma::mat A, arma::mat B)
{
	int N = A.n_rows;
	//printf("N=%f\n", N);
	arma::mat centroid_A = mean(A, 0);
	centroid_A = centroid_A.t(); //變成轉置矩陣後才能和centroid_A_2相乘
	arma::mat centroid_B = mean(B, 0);
	centroid_B = centroid_B.t(); //同centroid_A

	arma::mat centroid_A_2;
	centroid_A_2.ones(1, N); //產生1xN全1矩陣
	arma::mat centroid_A_3 = centroid_A * centroid_A_2;
	centroid_A_3 = centroid_A_3.t(); //因為剛剛轉置後才相乘，所以現在要轉置回來
									 // centroid_A_3 等於 python 的 tile(centroid_A, (N, 1))

	arma::mat AA = A - centroid_A_3;

	arma::mat centroid_B_2;
	centroid_B_2.ones(1, N); //產生1xN全1矩陣
	//centroid_B_2.print("centroid_B_2:");
	arma::mat centroid_B_3 = centroid_B * centroid_B_2;
	centroid_B_3 = centroid_B_3.t(); //轉置原因同centroid_A_3
									 // centroid_B_3 等於 tile(centroid_B, (N, 1))

	arma::mat BB = B - centroid_B_3;

	arma::mat H = AA.t() * BB;

	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	arma::svd(U, S, V, H);
	Vt = V.t();

	ret_R = Vt.t() * U.t();

	if (det(ret_R) < 0) {
		printf("%s\n", "Reflection detected\n");
		Vt.row(2) *= -1;
		ret_R = Vt.t() * U.t();
	}

	//ret_R.print("ret_R1:");
	//centroid_A.print("centroid_A:");

	ret_t = -ret_R * centroid_A + centroid_B; // 因為一開始centroid_A和centroid_B有轉置了，所以這邊不需要再轉置

}

=======

#include "MeshFusion.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>  
#include <armadillo>

using namespace std;
using namespace arma;


arma::mat ret_R;
arma::mat ret_t;
void rigid_transform_3D(arma::mat A, arma::mat B);


void MeshFusion::rigid_transformPose()
{

	arma::mat R = arma::randu<arma::mat>(3, 3) * 1; //產生3*3隨機矩陣，每個值的範圍為0~1
	arma::mat t = arma::randu<arma::mat>(3, 1) * 1; //經測試發現產生的隨機值好像是固定的
	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	svd(U, S, V, R);//對矩陣 R 進行奇異值分解 
	Vt = V.t();
	/*R.print("R:");
	U.print("U=\n");
	S.print("S=\n");
	V.print("V=\n");
	Vt.print("Vt=\n");*/
	R = U*Vt;
	//R.print("Rotation:");

	if (det(R) < 0) {
		Vt.row(2) *= -1;
		R = U*Vt;
	}


	int n = 10;

	arma::mat t2;
	t2.ones(1, n); //產生nx1全1矩陣
	arma::mat t3 = t * t2; // t3 等於 python 的 tile(t, (1, n))
	//t3.print("t3=\n");

	arma::mat A = arma::randu<arma::mat>(n, 3) * 1;
	//A.print("A=\n");
	arma::mat B = R * A.t() + t3; // t3 等於 python 的 tile(t, (1, n))
	//t3.print("t3:");
	B = B.t();

	rigid_transform_3D(A, B);


	arma::mat ret_t2;
	ret_t2.ones(1, n);//產生1xn全1矩陣
	//ret_t2.print("ret_t2:");
	arma::mat ret_t3 = ret_t * ret_t2;
	arma::mat A2 = (ret_R*A.t()) + ret_t3;//ret_t3 等於 tile(ret_t, (1, n))

	A2 = A2.t();

	arma::mat err = A2 - B;
	
	err = err % err; //點乘積 ,同 python 的 numpy.multiply()
	arma::mat c1 = sum(err, 1); //err各行的總和
	
	arma::mat r1 = sum(err, 0); //err各列的總和
	
	arma::mat r2 = sum(c1, 0);
	
	arma::mat c2 = sum(r1, 1);
	
	arma::mat err_sum = r2 + c2;
	//err_sum.print("err_sum:");
	A.print("Points A:");
	B.print("Points B:");
	R.print("Rotation:");
	t.print("Translation:");
	arma::mat rmse = sqrt(err_sum / n);
	rmse.print("rmse:");
	printf("If RMSE is near zero, the function is correct!\n");
	//system("pause");
}

void rigid_transform_3D(arma::mat A, arma::mat B)
{
	int N = A.n_rows;
	//printf("N=%f\n", N);
	arma::mat centroid_A = mean(A, 0);
	centroid_A = centroid_A.t(); //變成轉置矩陣後才能和centroid_A_2相乘
	arma::mat centroid_B = mean(B, 0);
	centroid_B = centroid_B.t(); //同centroid_A

	arma::mat centroid_A_2;
	centroid_A_2.ones(1, N); //產生1xN全1矩陣
	arma::mat centroid_A_3 = centroid_A * centroid_A_2;
	centroid_A_3 = centroid_A_3.t(); //因為剛剛轉置後才相乘，所以現在要轉置回來
									 // centroid_A_3 等於 python 的 tile(centroid_A, (N, 1))

	arma::mat AA = A - centroid_A_3;

	arma::mat centroid_B_2;
	centroid_B_2.ones(1, N); //產生1xN全1矩陣
	//centroid_B_2.print("centroid_B_2:");
	arma::mat centroid_B_3 = centroid_B * centroid_B_2;
	centroid_B_3 = centroid_B_3.t(); //轉置原因同centroid_A_3
									 // centroid_B_3 等於 tile(centroid_B, (N, 1))

	arma::mat BB = B - centroid_B_3;

	arma::mat H = AA.t() * BB;

	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	arma::svd(U, S, V, H);
	Vt = V.t();

	ret_R = Vt.t() * U.t();

	if (det(ret_R) < 0) {
		printf("%s\n", "Reflection detected\n");
		Vt.row(2) *= -1;
		ret_R = Vt.t() * U.t();
	}

	//ret_R.print("ret_R1:");
	//centroid_A.print("centroid_A:");

	ret_t = -ret_R * centroid_A + centroid_B; // 因為一開始centroid_A和centroid_B有轉置了，所以這邊不需要再轉置

}

>>>>>>> origin/master
