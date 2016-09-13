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

	arma::mat R = arma::randu<arma::mat>(3, 3) * 1; //����3*3�H���x�}�A�C�ӭȪ��d��0~1
	arma::mat t = arma::randu<arma::mat>(3, 1) * 1; //�g���յo�{���ͪ��H���Ȧn���O�T�w��
	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	svd(U, S, V, R);//��x�} R �i��_���Ȥ��� 
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
	t2.ones(1, n); //����nx1��1�x�}
	arma::mat t3 = t * t2; // t3 ���� python �� tile(t, (1, n))
	//t3.print("t3=\n");

	arma::mat A = arma::randu<arma::mat>(n, 3) * 1;
	//A.print("A=\n");
	arma::mat B = R * A.t() + t3; // t3 ���� python �� tile(t, (1, n))
	//t3.print("t3:");
	B = B.t();

	rigid_transform_3D(A, B);


	arma::mat ret_t2;
	ret_t2.ones(1, n);//����1xn��1�x�}
	//ret_t2.print("ret_t2:");
	arma::mat ret_t3 = ret_t * ret_t2;
	arma::mat A2 = (ret_R*A.t()) + ret_t3;//ret_t3 ���� tile(ret_t, (1, n))

	A2 = A2.t();

	arma::mat err = A2 - B;
	
	err = err % err; //�I���n ,�P python �� numpy.multiply()
	arma::mat c1 = sum(err, 1); //err�U�檺�`�M
	
	arma::mat r1 = sum(err, 0); //err�U�C���`�M
	
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
	centroid_A = centroid_A.t(); //�ܦ���m�x�}��~��Mcentroid_A_2�ۭ�
	arma::mat centroid_B = mean(B, 0);
	centroid_B = centroid_B.t(); //�Pcentroid_A

	arma::mat centroid_A_2;
	centroid_A_2.ones(1, N); //����1xN��1�x�}
	arma::mat centroid_A_3 = centroid_A * centroid_A_2;
	centroid_A_3 = centroid_A_3.t(); //�]�������m��~�ۭ��A�ҥH�{�b�n��m�^��
									 // centroid_A_3 ���� python �� tile(centroid_A, (N, 1))

	arma::mat AA = A - centroid_A_3;

	arma::mat centroid_B_2;
	centroid_B_2.ones(1, N); //����1xN��1�x�}
	//centroid_B_2.print("centroid_B_2:");
	arma::mat centroid_B_3 = centroid_B * centroid_B_2;
	centroid_B_3 = centroid_B_3.t(); //��m��]�Pcentroid_A_3
									 // centroid_B_3 ���� tile(centroid_B, (N, 1))

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

	ret_t = -ret_R * centroid_A + centroid_B; // �]���@�}�lcentroid_A�Mcentroid_B����m�F�A�ҥH�o�䤣�ݭn�A��m

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

	arma::mat R = arma::randu<arma::mat>(3, 3) * 1; //����3*3�H���x�}�A�C�ӭȪ��d��0~1
	arma::mat t = arma::randu<arma::mat>(3, 1) * 1; //�g���յo�{���ͪ��H���Ȧn���O�T�w��
	arma::mat U;
	arma::colvec S;
	arma::mat V;
	arma::mat Vt;
	svd(U, S, V, R);//��x�} R �i��_���Ȥ��� 
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
	t2.ones(1, n); //����nx1��1�x�}
	arma::mat t3 = t * t2; // t3 ���� python �� tile(t, (1, n))
	//t3.print("t3=\n");

	arma::mat A = arma::randu<arma::mat>(n, 3) * 1;
	//A.print("A=\n");
	arma::mat B = R * A.t() + t3; // t3 ���� python �� tile(t, (1, n))
	//t3.print("t3:");
	B = B.t();

	rigid_transform_3D(A, B);


	arma::mat ret_t2;
	ret_t2.ones(1, n);//����1xn��1�x�}
	//ret_t2.print("ret_t2:");
	arma::mat ret_t3 = ret_t * ret_t2;
	arma::mat A2 = (ret_R*A.t()) + ret_t3;//ret_t3 ���� tile(ret_t, (1, n))

	A2 = A2.t();

	arma::mat err = A2 - B;
	
	err = err % err; //�I���n ,�P python �� numpy.multiply()
	arma::mat c1 = sum(err, 1); //err�U�檺�`�M
	
	arma::mat r1 = sum(err, 0); //err�U�C���`�M
	
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
	centroid_A = centroid_A.t(); //�ܦ���m�x�}��~��Mcentroid_A_2�ۭ�
	arma::mat centroid_B = mean(B, 0);
	centroid_B = centroid_B.t(); //�Pcentroid_A

	arma::mat centroid_A_2;
	centroid_A_2.ones(1, N); //����1xN��1�x�}
	arma::mat centroid_A_3 = centroid_A * centroid_A_2;
	centroid_A_3 = centroid_A_3.t(); //�]�������m��~�ۭ��A�ҥH�{�b�n��m�^��
									 // centroid_A_3 ���� python �� tile(centroid_A, (N, 1))

	arma::mat AA = A - centroid_A_3;

	arma::mat centroid_B_2;
	centroid_B_2.ones(1, N); //����1xN��1�x�}
	//centroid_B_2.print("centroid_B_2:");
	arma::mat centroid_B_3 = centroid_B * centroid_B_2;
	centroid_B_3 = centroid_B_3.t(); //��m��]�Pcentroid_A_3
									 // centroid_B_3 ���� tile(centroid_B, (N, 1))

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

	ret_t = -ret_R * centroid_A + centroid_B; // �]���@�}�lcentroid_A�Mcentroid_B����m�F�A�ҥH�o�䤣�ݭn�A��m

}

>>>>>>> origin/master
