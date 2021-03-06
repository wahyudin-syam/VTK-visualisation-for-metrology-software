/*============================================================================================
This is a MEASURING ALGORITHM class
Filename: functionClass.h
Containing: Class definitiona and Class implementation
Description: 
- fitting algorithms, supporting functions, filtering algorithms, etc
- this class will be called by the Qt or other main window application to do data processing
- also contain random number generator based on: Park and Miller
==============================================================================================*/

//C/C++ LIB
#include <conio.h>
#include <stdio.h> //C I/O library
#include <iostream> //C++ I/O library
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
using namespace std;

//EIGEN LIB
#include <Eigen/Dense>
using namespace Eigen;

//For RANDOM NUMBER Generator of Uniform distribution
#define IA 16807 
#define IM 2147483647 
#define AM (1.0/IM) 
#define IQ 127773 
#define IR 2836 
#define NTAB 32 
#define NDIV (1+(IM-1)/NTAB) 
#define EPS 1.2e-7 
#define RNMX (1.0-EPS) 

//CLASS Definition
class myFunction{
	public:
	myFunction();
	~myFunction();

	//SUPPORTING Methods
	//Random number and error generator
	float randu(long *); //The uniform random number generator
	float randn(long *); //The Gaussian (normal) random number generator
	//VectorXd mvrnd(VectorXd, MatrixXd);  //Generating variates from multi-variate Gaussian distribution
	//MatrixXd mvrnd(MatrixXd, MatrixXd);  //Generating variates from multi-variate Gaussian distribution
	MatrixXd mvrnd(MatrixXd);  //Generating variates from multi-variate Gaussian distribution with mean 0
	MatrixXd errorSimulator(MatrixXd,int, double,double,double);

	//misceleneous functions
	void setRotoTranslationMatrix(MatrixXd &, double, double, double,double, double, double, double, double, double, double, double, double);
	void setTranslationMatrix(MatrixXd &, double, double, double);
	double r_norm_calc_sphere(MatrixXd , double,double,double,double);
	double r_norm_calc_cylinder(MatrixXd , double,double,double,double,double,double,double);
	double random_number();	
	MatrixXd plane_filter_3_sigma_based(MatrixXd);	
		//MatrixXd plane_filter_3_sigma_based(MatrixXd,double,double,double,double,double,double, int);
		//void plane_filter_3_sigma_based(MatrixXd, MatrixXd &, double *);
	double sigma_of_error_calc(MatrixXd,double,double,double,double,double,double, int); //sigma of error for plane
	void calculate_centroid(MatrixXd, double*, double*, double*);
	void find_farthest_point(MatrixXd, MatrixXd, double*, double*, double*);
	MatrixXd find_points_on_bottom_plane(MatrixXd, double, double, double, double, double, double);
	double dist_3D_point_to_line(double,double,double,double,double,double,double,double,double);
	double dist_3D_point_to_plane(double,double,double,double,double,double,double,double,double); //we sent all the plane parameter so that no refitting required
	double dist_3D_point_to_sphere(double,double,double,double,double,double,double); //we sent all the plane parameter so that no refitting required
	double dist_3D_point_to_cylinder(double,double,double,double,double,double,double,double,double,double); //we sent all the plane parameter so that no refitting required

	//GEOMETRIC FITTING Methods
	void chaos_initial_point_sphere(MatrixXd , double *, double *, double *, double *);
	void plane_fitting(MatrixXd , double *, double *, double *, double *, double *, double *);
	void line_fitting(MatrixXd , double *, double *, double *, double *, double *, double *);
	void sphere_fitting(MatrixXd , double *, double *, double *, double *, double*);
	void sphere_fitting_25_points(MatrixXd , MatrixXd , double *, double *, double *, double *);
	void cylinder_fitting(MatrixXd , double *, double *, double *, double *, double *,double *, double *, double*);

	//GEOMETRIC TOLERANCE VERIFICATION
	double straightness(MatrixXd , double *, double *, double *, double *, double *, double *);
	double flatness(MatrixXd , double *, double *, double *, double *, double *, double *);
	double sphereForm(MatrixXd , double *, double *, double *, double *);
	double cylindricity(MatrixXd , double *, double *, double *, double *, double *, double *,double *);

	//DATA FILTERING Methods
	MatrixXd calculate_row_col_zAvg(MatrixXd , int *, int *, double *);
	MatrixXd linear_gaussian_filter(MatrixXd , double);
	MatrixXd robust_gaussian_regression_filter(MatrixXd , double );
	MatrixXd test_return_matrix(MatrixXd ,int , int );
	double find_threshold(VectorXd ,int );
	double find_median(VectorXd &, int );
	double median_calculation(MatrixXd ,int , int ,int ,int ,int);
	double median_calculation2(MatrixXd ,int , int ,int ,int );
	double outlier_correction(MatrixXd ,int , int ,int ,int ,int, double , double, double );
	MatrixXd outlier_correction_filter(MatrixXd , double );
	MatrixXd outlier_correction_filter2(MatrixXd , double );
	MatrixXd outlier_correction_filter3(MatrixXd , double );
	MatrixXd sorting_matrix(MatrixXd , int *, int *);
	MatrixXd linear_gaussian_filter_calculation(MatrixXd , MatrixXd ,MatrixXd , int , int , double );
	MatrixXd robust_gaussian_regression_filter_calculation(MatrixXd ,MatrixXd ,MatrixXd , int , int , double );
	MatrixXd cylinder_linear_gaussian_filter(MatrixXd , double );
	MatrixXd cylinder_robust_gaussian_regression_filter(MatrixXd , double );

};


//CLASS Implementation
myFunction::myFunction(){
}

myFunction::~myFunction(){
}

//======================= RANDOM NUMBER GENERATOR =======================================
MatrixXd myFunction::errorSimulator(MatrixXd m,int mode, double s,double n,double r){
	//mode: 1-Gaussian, 2-Exponential, 3-Spherical
	//m is the data matriz (max. 3000 points)
	MatrixXd simulated_errors;
	MatrixXd sigma;
	
	int nPoints=0;
	nPoints=m.rows();

	//simulated_errors.resize(1,nPoint);
	sigma.resize(nPoints,nPoints);

	int i=0,j=0;
	double h=0.0, sum=0.0;
	switch(mode){// this is to build sigma: Variance-Covariance matrix, matrix in which the diagonal element is the variance and othe relement is the covariance
		case 1: //Gaussian Variogram
			for(i=0;i<nPoints;i++){
				for(j=i;j<nPoints;j++){// since it is symetric, we can copy index i,j and j,i
					sum=pow(m(i,0)-m(j,0),2)+pow(m(i,1)-m(j,1),2)+pow(m(i,2)-m(j,2),2);
					h=sqrt(sum);
					if(h==0){//the sill it self (the variance)
						sigma(i,j)=s; //The variance it self
					}
					else if(h<=r){
						sigma(i,j)=s-((s-n)*(1-exp(-3*(pow(h,2)))/(pow(r,2)))+n);
					}
					else{
						sigma(i,j)=0; //If the distance is ? r (range), they are considered independent (uncorrelated)
					}
					sigma(j,i)=sigma(i,j);
				}
			}
			break;
		case 2: //Exponential Variogram
			for(i=0;i<nPoints;i++){
				for(j=i;j<nPoints;j++){// since it is symetric, we can copy index i,j and j,i
					sum=pow(m(i,0)-m(j,0),2)+pow(m(i,1)-m(j,1),2)+pow(m(i,2)-m(j,2),2);
					h=sqrt(sum);
					if(h==0){//the sill it self (the variance)
						sigma(i,j)=s; //The variance it self
					}
					else if(h<=r){
						sigma(i,j)=s-((s-n)*(1-exp(-3*h)/r)+n);
					}
					else{
						sigma(i,j)=0; //If the distance is ? r (range), they are considered independent (uncorrelated)
					}
					sigma(j,i)=sigma(i,j);
				}
			}
			break;
		case 3: //Spherical Variogram
			for(i=0;i<nPoints;i++){
				for(j=i;j<nPoints;j++){// since it is symetric, we can copy index i,j and j,i
					sum=pow(m(i,0)-m(j,0),2)+pow(m(i,1)-m(j,1),2)+pow(m(i,2)-m(j,2),2);
					h=sqrt(sum);
					if(h==0){//the sill it self (the variance)
						sigma(i,j)=s; //The variance it self
					}
					else if(h<=r){
						sigma(i,j)=s-((s-n)*((3*h)/(2*r)-(pow(h,3))/(2*pow(r,3))));
					}
					else{
						sigma(i,j)=0; //If the distance is ? r (range), they are considered independent (uncorrelated)
					}
					sigma(j,i)=sigma(i,j);
				}
			}
			break;
		default://by default, use Gaussian variogram model
			for(i=0;i<nPoints;i++){
				for(j=i;j<nPoints;j++){// since it is symetric, we can copy index i,j and j,i
					sum=pow(m(i,0)-m(j,0),2)+pow(m(i,1)-m(j,1),2)+pow(m(i,2)-m(j,2),2);
					h=sqrt(sum);
					if(h==0){//the sill it self (the variance)
						sigma(i,j)=s; //The variance it self
					}
					else if(h<=r){
						sigma(i,j)=s-((s-n)*(1-exp(-3*(pow(h,2)))/(pow(r,2)))+n);
					}
					else{
						sigma(i,j)=0; //If the distance is ? r (range), they are considered independent (uncorrelated)
					}
					sigma(j,i)=sigma(i,j);
				}
			}
			break;
	}

	simulated_errors=mvrnd(sigma);
	return simulated_errors;
}

//VectorXd myFunction::mvrnd(VectorXd mean, MatrixXd sigma){
//MatrixXd myFunction::mvrnd(MatrixXd mean, MatrixXd sigma){
MatrixXd myFunction::mvrnd(MatrixXd sigma){ //mean is zero
	//This is to return a set of errors with mean 0. the errors are in a single row vector (max. 3000 errors at once)
	//The general procedure is how to extract matrix U, which is a upper triangle matrix, resulted from
	//Cholesky decomposition of matrix sigma (variance-covariance matrix). and generate the
	//multi-variat random from s=randn*U+mu
	//lets define the vector is a 1xn matrix

	//VectorXd errors;
	MatrixXd errors; //matrix (1xn)
	MatrixXd U; //upper triangulation matrix (nxn) resulted from choleski decomposition of matrix sigma
	MatrixXd mu;//matrix (1xn) = all zeros
	MatrixXd m_randn; //matrix (1xn)
	
	int n=0;
	n=sigma.rows(); //determining number of points

	//allocating size to each matrix
	errors.resize(1,n);
	mu.resize(1,n);
	m_randn.resize(1,n);
	U.resize(n,n); 
	
	int i=0;
	//Initializing zero matrix mu
	for(i=0;i<n;i++){
		mu(0,i)=0.0;
	}

	//initializing m_randn matrix
	long initial=-1000000; //should be a negative integer
	for(i=0;i<n;i++){
		m_randn(0,i)=randn(&initial);
	}
	
	//We are using robust choleski decomposition to get matrix U from matirx sigma.
	//Robust Cholesky means that we applied cholesky decomposition to all type of matrix, not only to
	//Positive-Definite matrix (but also to positive and negative semi-definite matrix
	//So, we do not need to do checking whetehr the matric is positive-definite or not.

	LDLT< MatrixXd, Lower > chol_robust(sigma);
	U=chol_robust.matrixU();

	//CALCULATING the simulated errors
	errors=m_randn*U+mu;
	
	return errors;
}

float   myFunction::randn(long   *idum){ 
	/*Returns a normally distributed deviate with zero mean and unit variance, using ran1(idum)
	as the source of uniform deviates.*/
	//float randu(long *idum);
	randu(idum);
	static int iset=0;
	static float gset;
	float fac,rsq,v1,v2;
	if (*idum < 0) iset=0; //Reinitialize.
	if (iset == 0) { //We don’t have an extra deviate handy, so
		do {
			v1=2.0*randu(idum)-1.0; //pick two uniform numbers in the square ex-
								   //tending from -1 to +1 in each direction, 
			v2=2.0*randu(idum)-1.0;
			rsq=v1*v1+v2*v2;	   //see if they are in the unit circle,

		}while (rsq >= 1.0 || rsq == 0.0); //and if they are not, try again.
		fac=sqrt(-2.0*log(rsq)/rsq);
		//Now make the Box-Muller transformation to get two normal deviates. Return one and
		//save the other for next time.
		gset=v1*fac;
		iset=1; //Set ﬂag.
		return v2*fac;
	} 
	else { //We have an extra deviate handy,
		iset=0; //so unset the ﬂag,
		return gset; //and return it.
	}
}

float   myFunction::randu(long   *idum){ 
/*“Minimal” random  number  generator  of  Park and  Miller with  Bays-Durham  shuﬄe  and  added 
safeguards.  Returns  a  uniform  random  deviate between  0.0 and  1.0 (exclusive of  the  endpoint 
values). Call  with  idum a  negative  integer  to  initialize;  thereafter,  do  not  alter  idum between 
successive  deviates  in  a  sequence. RNMX  should  a  roximate  the  largest  ﬂoating  value  that  is 
less than  1. */
     int  j; 
     long  k; 
     static   long  iy=0; 
     static   long  iv[NTAB]; 
     float  temp; 

     if  (*idum   <= 0  ||  !iy)  {			//Initialize               
          if  (-(*idum)   <  1)  *idum=1;   //Be sure to prevent idum=0           
          else  *idum   = -(*idum); 
          for  (j=NTAB+7;j>=0;j--)      {   //load the suffle table (after 8 warms up)           
              k=(*idum)/IQ; 
               *idum=IA*(*idum-k*IQ)-IR*k; 
               if (*idum   <  0)  *idum  +=  IM; 
               if (j  <  NTAB)  iv[j]   = *idum; 
          } 
          iy=iv[0]; 
     } 

     k=(*idum)/IQ;                        //start here when not initializing           
     *idum=IA*(*idum-k*IQ)-IR*k;		  //Compute idum=(IA*idum) %IM without overflow by Scharge's method                  
     if  (*idum   < 0)  *idum   += IM;                    
     j=iy/NDIV;							  //will be in the range 0..NTAB-                           
     iy=iv[j];                            //output previously stored value and refill the suffle table       .    
     iv[j]   = *idum;                                     
     if  ((temp=AM*iy)     > RNMX)   return   RNMX;   //Because users do not expect endpoint values
     else  return   temp; 
} 

//====================== SETTING ROTO- and TRANSLATION MATRIX ===========================
void myFunction::setRotoTranslationMatrix(MatrixXd &T, double dx, double dy, double dz,double c11, double c12, double c13, double c21, double c22, double c23, double c31, double c32, double c33){
	T(0,0)=c11; T(0,1)=c12; T(0,2)=c13; T(0,3)=dx;
	T(1,0)=c21; T(1,1)=c22; T(1,2)=c23; T(1,3)=dy;
	T(2,0)=c31; T(2,1)=c32; T(2,2)=c33; T(2,3)=dz;
	T(3,0)=0;   T(3,1)=0;   T(3,2)=0;   T(3,3)=1;
}

void myFunction::setTranslationMatrix(MatrixXd &T, double dx, double dy, double dz){
	T(0,0)=1; T(0,1)=0; T(0,2)=0; T(0,3)=dx;
	T(1,0)=0; T(1,1)=1; T(1,2)=0; T(1,3)=dy;
	T(2,0)=0; T(2,1)=0; T(2,2)=1; T(2,3)=dz;
	T(3,0)=0; T(3,1)=0; T(3,2)=0; T(3,3)=1;
}

//======================= OTHERS ALGORITHM ===================================

MatrixXd myFunction::find_points_on_bottom_plane(MatrixXd m, double px, double py, double pz, double n1, double n2, double n3){
	//INPUT: Original matrix points and Robust fitted plane parameter (TOP Plane) as reference plane
	//OUTPUT: Points on bottom plane, which is points lower than the TOP Plane and dist < 3sigma of the top plane,
	//		  because, we have to avoid point on edge, which is prone to outliers

	MatrixXd m_bottom_plane;

	//FINDING THE SIGMA
	int counterPoint=0;
	counterPoint=m.rows();

	//Calculating the sigma of residual or error
	double sigma=0.0,three_sigma=0.0;
	sigma=sigma_of_error_calc(m,px,py,pz,n1,n2,n3,counterPoint);
	three_sigma=3*sigma;

	//SELECTING THE POINTS ON BOTTOM PLANE
	int i=0, counter=0;
	double dist=0.0;
	MatrixXd mTemp;
	mTemp.resize(counterPoint,3);

	for(i=0;i<counterPoint;i++){
		dist=dist_3D_point_to_plane(m(i,0),m(i,1),m(i,2),px,py, pz,  n1, n2, n3);
		if(dist<three_sigma && m(i,2)<pz){ //select the point with distance <3sigma and z lower than TOP PLANE
			mTemp(counter,0)=m(i,0);
			mTemp(counter,1)=m(i,1);
			mTemp(counter,2)=m(i,2);
			counter++;
		}
	}
	//copy the data to m_bottom_plane;
	m_bottom_plane.resize(counter,3);
	for(i=0;i<counter;i++){
		m_bottom_plane(i,0)=mTemp(i,0);
		m_bottom_plane(i,1)=mTemp(i,1);
		m_bottom_plane(i,2)=mTemp(i,2);
	}

	return m_bottom_plane;
}

void myFunction::calculate_centroid(MatrixXd m, double *px_centroid, double *py_centroid, double *pz_centroid){
	//INPUT: Matrix of points
	//OUTPUT: cantroudn of the point cloud (average X, Y and Z)

	int counter=0, i=0;
	counter=m.rows();

	double sumX, sumY, sumZ;

	sumX=0.0;
	sumY=0.0;
	sumZ=0.0;

	for(i=0;i<counter;i++){
		sumX=sumX+m(i,0);
		sumY=sumY+m(i,1);
		sumZ=sumZ+m(i,2);
	}

	*px_centroid=sumX/counter;
	*py_centroid=sumY/counter;
	*pz_centroid=sumZ/counter;
}

double myFunction::dist_3D_point_to_line(double px_far,double py_far,double pz_far,double px,double py,double pz,double n1,double n2,double n3){
	double dist=0.0;
	double xi_min_x=0.0, a_dot_xi_min_x=0.0;
	xi_min_x=(px_far-px)*(px_far-px)+(py_far-py)*(py_far-py)+(pz_far-pz)*(pz_far-pz);
	a_dot_xi_min_x=(n1*(px_far-px)+n2*(py_far-py)+n3*(pz_far-pz))*(n1*(px_far-px)+n2*(py_far-py)+n3*(pz_far-pz));
	dist=sqrt(xi_min_x-a_dot_xi_min_x);
	return dist;
}

double myFunction::dist_3D_point_to_plane(double px_far,double py_far,double pz_far,double px,double py, double pz, double n1,double n2,double n3){
	double dist=0.0;
	double err=0.0;
	err=n1*(px_far-px)+n2*(py_far-py)+n3*(pz_far-pz);
	dist=fabs(err);
	return dist;
}

double myFunction::dist_3D_point_to_sphere(double px_far,double py_far,double pz_far,double px,double py,double pz,double r){
	double dist=0.0;
	dist=sqrt((px_far-px)*(px_far-px)+(py_far-py)*(py_far-py)+(pz_far-pz)*(pz_far-pz))-r;
	return dist;
}

double myFunction::dist_3D_point_to_cylinder(double px_far,double py_far,double pz_far,double px,double py,double pz,double n1, double n2, double n3, double r){
	//ref: Shakarji 1998
	double dist=0.0;
	double a[3], p0[3];
	double lengthA=0.0, uSquare=0.0, vSquare=0.0, wSquare=0.0, f=0.0;
	
	//assigning value
	p0[0]=px;
	p0[1]=py;
	p0[2]=pz;
	p0[3]=n1;
	p0[4]=n2;
	p0[5]=n3;
	p0[6]=r;

	//calculate vector a=A/|A|
	lengthA=sqrt(p0[3]*p0[3]+p0[4]*p0[4]+p0[5]*p0[5]);
	a[0]=p0[3]/lengthA; //a
	a[1]=p0[4]/lengthA; //b
	a[2]=p0[5]/lengthA; //c

	//calculate fi=sqrt(u^2+v^2+w^2) --> scalar
	/*uSquare=(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]))*(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]));
	vSquare=(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]))*(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]));
	wSquare=(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]))*(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]));*/
	uSquare=(a[2]*(py_far-p0[1])-a[1]*(pz_far-p0[2]))*(a[2]*(py_far-p0[1])-a[1]*(pz_far-p0[2]));
	vSquare=(a[0]*(pz_far-p0[2])-a[2]*(px_far-p0[0]))*(a[0]*(pz_far-p0[2])-a[2]*(px_far-p0[0]));
	wSquare=(a[1]*(px_far-p0[0])-a[0]*(py_far-p0[1]))*(a[1]*(px_far-p0[0])-a[0]*(py_far-p0[1]));	
	f=sqrt(uSquare+vSquare+wSquare);

	//Build distance vector d
	dist=f-p0[6];
	return dist;
}

void myFunction::find_farthest_point(MatrixXd m2,MatrixXd m, double *px_far, double *py_far, double *pz_far){
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;
	int counterPoint=0;

	plane_fitting(m2,&px,&py,&pz,&n1,&n2,&n3);//fitting parameter from fulterd points
	counterPoint=m.rows(); //distance calculation from all points

	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0;
	double dist=0.0;
	int i;
	VectorXd err(counterPoint);

	*px_far=0.0;
	*py_far=0.0;
	*pz_far=0.0;

	//finding the farthest point
	for(i=0;i<counterPoint;i++){
		err(i)=n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz);
		A=n1;
		B=n2;
		C=n3;
		D=-1*(n1*px+n2*py+n3*pz);
		plane=A*m(i,0)+B*m(i,1)+C*m(i,2)+D;
		if(i==0){
			dist=fabs(plane);
			*px_far=m(i,0);
			*py_far=m(i,1);
			*pz_far=m(i,2);
		}
		else{
			if(fabs(plane)>dist){ //current point is farther then previous recorded point
				dist=fabs(plane);
				*px_far=m(i,0);
				*py_far=m(i,1);
				*pz_far=m(i,2);
			}
		}		
	}	
}

//MatrixXd myFunction::plane_filter_3_sigma_based(MatrixXd m,double px,double py,double pz,double n1,double n2,double n3, int counterPoint){
MatrixXd myFunction::plane_filter_3_sigma_based(MatrixXd m){
//void myFunction::plane_filter_3_sigma_based(MatrixXd m, MatrixXd &m2, double *num_of_filtered_points){
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0;
	int counterPoint=0;

	plane_fitting(m,&px,&py,&pz,&n1,&n2,&n3);
	counterPoint=m.rows();

	MatrixXd m2;
	MatrixXd mTemp;
	VectorXd err(counterPoint);
	int i;

	//Calculating the sigma of residual or error
	double sigma=0.0,three_sigma=0.0;
	sigma=sigma_of_error_calc(m,px,py,pz,n1,n2,n3,counterPoint);
	three_sigma=3*sigma;
	//removing outliers and calculate the filtered points
	int outlier_counter=0;
	int counterPoint2=0;
	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0;
	mTemp.resize(counterPoint,3);
	for(i=0;i<counterPoint;i++){
		err(i)=n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz);
		A=n1;
		B=n2;
		C=n3;
		D=-1*(n1*px+n2*py+n3*pz);
		plane=A*m(i,0)+B*m(i,1)+C*m(i,2)+D;
		if(fabs(plane)<=three_sigma){ //record the points if the point distance < 3 sigma
			mTemp(counterPoint2,0)=m(i,0);
			mTemp(counterPoint2,1)=m(i,1);
			mTemp(counterPoint2,2)=m(i,2);
			counterPoint2++;
		}
		else{
			outlier_counter++;
		}
	}
	
	//puting the good point into m2 matrix
	m2.resize(counterPoint2,3);
	
	for(i=0;i<counterPoint2;i++){
		m2(i,0)=mTemp(i,0);
		m2(i,1)=mTemp(i,1);
		m2(i,2)=mTemp(i,2);
	}
	//*num_of_filtered_points=counterPoint2;
	err.resize(counterPoint2);

	return m2;
}

double myFunction::sigma_of_error_calc(MatrixXd m,double px,double py,double pz,double n1,double n2,double n3, int counterPoint){
	double sigma=0.0;

	// Calculation of for sigma of the error
	double norm_of_residual=0.0, sigma_of_error=0.0, mean_of_error=0.0, min_error=0.0, max_error=0.0;
	double sigma_of_x, sigma_of_y, sigma_of_z;
	double point_on_plane[3], normal_vector[3];
	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0,errMax=0.0, errMin=0.0, sumX=0.0, sumY=0.0, sumZ=0.0;
	int i;
	VectorXd err(counterPoint);

	for(i=0;i<counterPoint;i++){
		err(i)=n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz);
		A=n1;
		B=n2;
		C=n3;
		D=-1*(n1*px+n2*py+n3*pz);
		plane=A*m(i,0)+B*m(i,1)+C*m(i,2)+D;
		if(plane>0){ //the point on the side of normal vector direction
			if(errMax<err(i) ){
				errMax=err(i);
			}
		}
		else if(plane<0){//the point on the OPPOSITE side of normal vector direction
			if(errMin<fabs(err(i)) ){
				errMin=fabs(err(i));
			}
		}
		norm_of_residual=norm_of_residual+err(i)*err(i);		
	}	
	mean_of_error=err.mean();	

	double ss_Error=0.0,ss_X=0.0,ss_Y=0.0,ss_Z=0.0;
	for(i=0;i<counterPoint;i++){
		ss_Error=ss_Error+(err(i)-mean_of_error)*(err(i)-mean_of_error);		
	}
	
	sigma=sqrt(ss_Error/(counterPoint-1));

	return sigma;
}

//======================= GEOMETRIC TOLERANCE VERIFICATION ============================
double myFunction::straightness(MatrixXd m, double *p_x, double *p_y, double *p_z, double *n_1, double *n_2, double *n_3){
	//NOTE: filtering is needed in the future before form error calculation
	double px=0.0,py=0.0, pz=0.0, n1=0.0, n2=0.0, n3=0.0;
	double straightness=0.0;
	
	//fitting a LS line
	line_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	//return value by reference
	*p_x=px;
	*p_y=py;
	*p_z=pz;
	*n_1=n1;
	*n_2=n2;
	*n_3=n3;
	
	int counterPoint=0;
	counterPoint=m.rows();

	//calculating straightness
	//NOTE: filtering is needed in the future before form error calculation
	int i;
	double norm_of_residual=0.0, sigma_of_error=0.0, mean_of_error=0.0, min_error=0.0, max_error=0.0;
	double xi_min_x=0.0, a_dot_xi_min_x=0.0;
	double sigma_of_x=0.0, sigma_of_y=0.0, sigma_of_z=0.0;
	double point_on_plane[3], normal_vector[3];
	double A=0.0,B=0.0,C=0.0,D=0.0,plane,errMax=0.0, errMin=0.0, sumX=0.0, sumY=0.0, sumZ=0.0;
	VectorXd err(counterPoint);
	
	for(i=0;i<counterPoint;i++){
		xi_min_x=(m(i,0)-px)*(m(i,0)-px)+(m(i,1)-py)*(m(i,1)-py)+(m(i,2)-pz)*(m(i,2)-pz);
		a_dot_xi_min_x=(n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz))*(n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz));
		err(i)=sqrt(xi_min_x-a_dot_xi_min_x);
			if(errMax<err(i)){
				errMax=err(i);
			}	
			if(errMin>err(i)){
				errMin=err(i);
			}		
		norm_of_residual=norm_of_residual+err(i)*err(i);
		sumX=sumX+m(i,0);
		sumY=sumY+m(i,1);
		sumZ=sumZ+m(i,2);
	}
	norm_of_residual=sqrt(norm_of_residual);
	mean_of_error=err.mean();
	max_error=errMax;
	min_error=errMin;
	point_on_plane[0]=px;
	point_on_plane[1]=py;
	point_on_plane[2]=pz;
	normal_vector[0]=n1;
	normal_vector[1]=n2;
	normal_vector[2]=n3;

	double ss_Error=0.0,ss_X=0.0,ss_Y=0.0,ss_Z=0.0;
	for(i=0;i<counterPoint;i++){
		ss_Error=ss_Error+(err(i)-mean_of_error)*(err(i)-mean_of_error);
		ss_X=ss_X+(m(i,0)-px)*(m(i,0)-px);
		ss_Y=ss_Y+(m(i,1)-py)*(m(i,1)-py);
		ss_Z=ss_Z+(m(i,2)-pz)*(m(i,2)-pz);
	}
	sigma_of_error=sqrt(ss_Error/(counterPoint-1));
	sigma_of_x=sqrt(ss_X/(counterPoint-1));
	sigma_of_y=sqrt(ss_Y/(counterPoint-1));
	sigma_of_z=sqrt(ss_Z/(counterPoint-1));

	straightness=2*max_error;
	return straightness;
}

double myFunction::flatness(MatrixXd m, double *p_x, double *p_y, double *p_z, double *n_1, double *n_2, double *n_3){
	double flatness=0.0;
	
	//Plane fitting
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0; //paramater for the plane: point on plane and plane normal vector
	plane_fitting(m,&px,&py,&pz,&n1,&n2,&n3);

	//return value by reference
	*p_x=px;
	*p_y=py;
	*p_z=pz;
	*n_1=n1;
	*n_2=n2;
	*n_3=n3;

	int counterPoint=0;
	counterPoint=m.rows();

	// Calculation of for sigma of the error
	//NOTE: filtering is needed in the future before form error calculation
	int i;
	double norm_of_residual=0.0, sigma_of_error=0.0, mean_of_error=0.0, min_error=0.0, max_error=0.0;
	double sigma_of_x, sigma_of_y, sigma_of_z;
	double point_on_plane[3], normal_vector[3];
	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0,errMax=0.0, errMin=0.0, sumX=0.0, sumY=0.0, sumZ=0.0;
	VectorXd err(counterPoint);

	for(i=0;i<counterPoint;i++){
		err(i)=n1*(m(i,0)-px)+n2*(m(i,1)-py)+n3*(m(i,2)-pz);
		A=n1;
		B=n2;
		C=n3;
		D=-1*(n1*px+n2*py+n3*pz);
		plane=A*m(i,0)+B*m(i,1)+C*m(i,2)+D;
		if(plane>0){ //the point on the side of normal vector direction
			if(errMax<err(i) ){
				errMax=err(i);
			}
		}
		else if(plane<0){//the point on the OPPOSITE side of normal vector direction
			if(errMin<fabs(err(i)) ){
				errMin=fabs(err(i));
			}
		}
		norm_of_residual=norm_of_residual+err(i)*err(i);		
	}	
	mean_of_error=err.mean();	

	double ss_Error=0.0,ss_X=0.0,ss_Y=0.0,ss_Z=0.0;
	for(i=0;i<counterPoint;i++){
		ss_Error=ss_Error+(err(i)-mean_of_error)*(err(i)-mean_of_error);		
	}
	
	flatness=errMin+errMax;
	return flatness;
}

double myFunction::sphereForm(MatrixXd m, double *p_x, double *p_y, double *p_z, double *r_fit){
	double sphereForm=0.0;

	//sphere form error calculation
		
	//sphere fitting
	double px=0.0,py=0.0,pz=0.0,r=0.0,r_norm=0.0; //paramater for the plane: point on plane and plane normal vector
	sphere_fitting(m,&px,&py,&pz,&r,&r_norm);

	//return value by reference
	*p_x=px;
	*p_y=py;
	*p_z=pz;
	*r_fit=r;

	int counterPoint=0;
	counterPoint=m.rows();

	// Calculation of for sigma of the error
	//NOTE: filtering is needed in the future before form error calculation
	int i;
	double norm_of_residual=0.0, sigma_of_error=0.0, mean_of_error=0.0, min_error=0.0, max_error=0.0;
	double sigma_of_x, sigma_of_y, sigma_of_z;
	double point_on_plane[3], normal_vector[3];
	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0,errMax=0.0, errMin=0.0, sumX=0.0, sumY=0.0, sumZ=0.0;
	VectorXd err(counterPoint);

	for(i=0;i<counterPoint;i++){
		err(i)=dist_3D_point_to_sphere(m(i,0),m(i,1),m(i,2),px,py,pz,r);
		
		if(err(i)>0){ //the point on the side of normal vector direction
			if(errMax<err(i) ){
				errMax=err(i);
			}
		}
		else if(err(i)<0){//the point on the OPPOSITE side of normal vector direction
			if(errMin<fabs(err(i)) ){
				errMin=fabs(err(i));
			}
		}
		norm_of_residual=norm_of_residual+err(i)*err(i);		
	}	
	mean_of_error=err.mean();	

	double ss_Error=0.0,ss_X=0.0,ss_Y=0.0,ss_Z=0.0;
	for(i=0;i<counterPoint;i++){
		ss_Error=ss_Error+(err(i)-mean_of_error)*(err(i)-mean_of_error);		
	}
	
	sphereForm=errMin+errMax;

	return sphereForm;
}

double myFunction::cylindricity(MatrixXd m, double *p_x, double *p_y, double *p_z, double *n_1, double *n_2, double *n_3, double *r_fit){
	double cylindricity=0.0;

	//cylindricity calculation
		
	//cylinder fitting
	double px=0.0,py=0.0,pz=0.0,n1=0.0,n2=0.0,n3=0.0,r=0.0,r_norm=0.0; //paramater for the plane: point on plane and plane normal vector
	cylinder_fitting(m,&px,&py,&pz,&n1,&n2,&n3,&r,&r_norm);

	//return value by reference
	*p_x=px;
	*p_y=py;
	*p_z=pz;
	*n_1=n1;
	*n_2=n2;
	*n_3=n3;
	*r_fit=r;

	int counterPoint=0;
	counterPoint=m.rows();

	// Calculation of for sigma of the error
	//NOTE: filtering is needed in the future before form error calculation
	int i;
	double norm_of_residual=0.0, sigma_of_error=0.0, mean_of_error=0.0, min_error=0.0, max_error=0.0;
	double sigma_of_x, sigma_of_y, sigma_of_z;
	double point_on_plane[3], normal_vector[3];
	double A=0.0,B=0.0,C=0.0,D=0.0,plane=0.0,errMax=0.0, errMin=0.0, sumX=0.0, sumY=0.0, sumZ=0.0;
	VectorXd err(counterPoint);

	for(i=0;i<counterPoint;i++){
		err(i)=dist_3D_point_to_cylinder(m(i,0),m(i,1),m(i,2),px,py,pz,n1,n2,n3,r);
		
		if(err(i)>0){ //the point on the side of normal vector direction
			if(errMax<err(i) ){
				errMax=err(i);
			}
		}
		else if(err(i)<0){//the point on the OPPOSITE side of normal vector direction
			if(errMin<fabs(err(i)) ){
				errMin=fabs(err(i));
			}
		}
		norm_of_residual=norm_of_residual+err(i)*err(i);		
	}	
	mean_of_error=err.mean();	

	double ss_Error=0.0,ss_X=0.0,ss_Y=0.0,ss_Z=0.0;
	for(i=0;i<counterPoint;i++){
		ss_Error=ss_Error+(err(i)-mean_of_error)*(err(i)-mean_of_error);		
	}
	
	cylindricity=errMin+errMax;

	return cylindricity;
}



//======================= CHAOS ALGORITHM ===================================
//Using Chaotic Optimization Algorithm
//function to improve the initial point by chaos optimization algorithm
double myFunction::r_norm_calc_cylinder(MatrixXd m, double p01,double p02,double p03,double p04,double p05,double p06,double p07){
	//Called by the chaos algorithm
	double rnorm=0.0, dist=0.0, p0[7];
	double a[3];
	double uSquare=0.0, vSquare=0.0, wSquare=0.0, g=0.0, f=0.0, d=0.0, lengthA=0.0;

	//value assignment
	p0[0]=p01;
	p0[1]=p02;
	p0[2]=p03;
	p0[3]=p04;
	p0[4]=p05;
	p0[5]=p06;
	p0[6]=p07;

	//r norm calculation
	int counterPoint=0, i=0;
	counterPoint=m.rows();		
	//calculate vector a=A/|A|
	lengthA=sqrt(p0[3]*p0[3]+p0[4]*p0[4]+p0[5]*p0[5]);
	a[0]=p0[3]/lengthA; //a
	a[1]=p0[4]/lengthA; //b
	a[2]=p0[5]/lengthA; //c
	rnorm=0.0;
	for(i=0;i<counterPoint;i++){
		//calculate gi=a.(xi-x) --> scalar
		g=a[0]*(m(i,0)-p0[0])+a[1]*(m(i,1)-p0[1])+a[2]*(m(i,2)-p0[2]);
		//calculate fi=sqrt(u^2+v^2+w^2) --> scalar
		uSquare=(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]))*(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]));
		vSquare=(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]))*(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]));
		wSquare=(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]))*(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]));
		f=sqrt(uSquare+vSquare+wSquare);
		//Build distance vector d
		d=f-p0[6];
		//d(i)=fabs(f(i)-p0[6]);
		//calculate J0=sum (d^2) --> sum of square
		rnorm=rnorm+d*d;
	}

	return rnorm;
}

double myFunction::r_norm_calc_sphere(MatrixXd m, double p01,double p02,double p03,double p04){
	//Called by the chaos algorithm
	double rnorm=0.0, dist=0.0, p0[4];
	p0[0]=p01;
	p0[1]=p02;
	p0[2]=p03;
	p0[3]=p04;
	
	int counterPoint=0, i=0;
	counterPoint=m.rows();	
	rnorm=0.0;
	for(i=0; i<counterPoint; i++){
		dist=sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]))-p0[3];
		rnorm=rnorm+dist*dist;
	}	
	return rnorm;
}

double myFunction::random_number(){//Sjould be improved and changed
	double x1,x2,x3,x4,x5,x6,x7, num;
	//randomize();
	srand ( time(0) );
	x1=(double)(rand() % 100)/100; //we need to convert from int to double
	x2=(double)(rand()%100)/100;
	x3=(double)(rand()%100)/100;
	x4=(double)(rand()%100)/100;
	x5=(double)(rand()%100)/100;
	x6=(double)(rand()%100)/100;
	x7=(double)(rand()%100)/100;
	num=(x1+x2+x3+x4+x5+x6+x7)/7;
	//printf("\n%lf\n",x1);
	return num;
}

void myFunction::chaos_initial_point_sphere(MatrixXd m, double *p01, double *p02, double *p03, double *p04){
	int counterPoint=0, k=0 , r=0, kMax=0, rMax=0, paramDim=4;
	int i=0;
	double limit=0.0, ub=0.0, lb=0.0, lamda=0, gamma=0.0, jstar=0.0, jk=0.0, rnorm=0.0;
	double t0=0.0, tk[4], tstar[4], a[4], b[4], ar[4], br[4], pstar[4], pk[4];
	double arOld[4], arNew[4], brOld[4], brNew[4], p0[4];
	long seed=-1; //seed should be a negative integer

	p0[0]=*p01;
	p0[1]=*p02;
	p0[2]=*p03;
	p0[3]=*p04;

	counterPoint=m.rows();
	limit=0.000005;
	ub=limit;
	lb=-limit;

	k=0;
	r=0;
	kMax=10;//80;
	rMax=10;//80;

	lamda=4;
	seed=-1000;
	
	//------------------ STEP 1: INITIALIZATION ---------------------------
	
	for(i=0;i<paramDim;i++){
		//t0=random_number();
		t0=randu(&seed);
		while(t0==0.0|| t0==0.25 || t0==0.5 || t0==0.75 || t0==1.0){ //possible infinite loop
			//t0=random_number();
			t0=randu(&seed);
			
		}
		tk[i]=t0;
		tstar[i]=t0;
		a[i]=p0[i]-limit; //initial bounding box for parameter
		b[i]=p0[i]+limit;

		ar[i]=a[i];
		br[i]=b[i];

		pstar[i]=p0[i];
	}
	pstar[3]=fabs(pstar[3]);//radius is always positive

	jstar=r_norm_calc_sphere(m, p0[0],p0[1],p0[2],p0[3]);	
	gamma=0.45;
	
	while(r<rMax){
		while(k<kMax){
		//------------------STEP 2: MAPPING VARIABLES --------------------------            
			for(i=0;i<paramDim;i++){
				pk[i]=ar[i]+tk[i]*(br[i]-ar[i]);
			}
			//pk[0]=fixPx;
			//pk[1]=fixPy;
			pk[3]=fabs(pk[3]);//radius is always positive

		//------------------STEP 3: BEST SO FAR --------------------------------  
			jk=r_norm_calc_sphere(m, pk[0],pk[1],pk[2],pk[3]);	//r_norm_calc_sphere(m, pk);
			//printf("jk=%lf\n",jk);
			if(jk<jstar){
				jstar=jk;
				for(i=0;i<paramDim;i++){
					pstar[i]=pk[i];
					tstar[i]=tk[i];
				}
			}

		//------------------STEP 4: CHAOS VARIABLE ITERATION ------------------
			k=k+1;
			for(i=0;i<paramDim;i++){
				tk[i]=4*tk[i]*(1-tk[i]);
			}
		//------------------STEP 5: CHECK --------------------------------------
		}
		r=r+1;

		//------------------STEP 6: CHANGE SEARCH RANGE ------------------------
		for(i=0;i<paramDim;i++){
			arOld[i]=ar[i];
			brOld[i]=br[i];
			arNew[i]=pstar[i]-gamma*(br[i]=ar[i]);
			brNew[i]=pstar[i]+gamma*(br[i]=ar[i]);

			if(arNew[i]<arOld[i]){
				ar[i]=arNew[i];
			}
			if(brNew[i]>brOld[i]){
				br[i]=brNew[i];
			}
		}

		//------------------STEP 7: STOP OR REPEAT -----------------------------
		if(r<rMax){
			k=0;
			for(i=0;i<paramDim;i++){
				//t0=random_number();
				t0=randu(&seed);
				while(t0==0 || t0==0.25 || t0==0.5 || t0==0.75 || t0==1){
					//t0=random_number();
					t0=randu(&seed);
				}
				tk[i]=t0;				
			}
		}

	}

	for(i=0;i<paramDim;i++){
		p0[i]=pstar[i];
	}

	*p01=p0[0];
	*p02=p0[1];
	*p03=p0[2];
	*p04=p0[3];

	
}

// ==================== FITTING ALGORITHM ====================================
void myFunction::plane_fitting(MatrixXd m, double *px, double *py, double *pz, double *n1, double *n2, double *n3){
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;

	counterPoint=m.rows();

	//calculating the points centroid	
	for(i=0;i<counterPoint;i++){
		xi=m(i,0);
		yi=m(i,1);
		zi=m(i,2);			
		sumX=sumX+xi;
		sumY=sumY+yi;
		sumZ=sumZ+zi;
	}
	avgX=sumX/counterPoint;
	avgY=sumY/counterPoint;
	avgZ=sumZ/counterPoint;

	//Transforming the points to their centroid
	MatrixXd m_hom(counterPoint,4);
	for(i=0;i<counterPoint;i++){
		m_hom(i,0)=m(i,0);
		m_hom(i,1)=m(i,1);
		m_hom(i,2)=m(i,2);
		m_hom(i,3)=1;
	}
	MatrixXd m_hom_trans(4,counterPoint);
	m_hom_trans=m_hom.transpose();
	MatrixXd m_hom_trans2(4,counterPoint);
	
	MatrixXd T(4,4);
	setTranslationMatrix(T,-avgX, -avgY,-avgZ);
	//printf("\nTranslation Matrix: \n");
	//cout<<T;
	
	m_hom_trans2=T*m_hom_trans;

	MatrixXd m_translated(counterPoint,3);
	for(i=0;i<counterPoint;i++){
		m_translated(i,0)=m_hom_trans2(0,i);
		m_translated(i,1)=m_hom_trans2(1,i);
		m_translated(i,2)=m_hom_trans2(2,i);		
	}

	//SVD calculation of a rectangular matrix
	//MatrixXd U(counterPoint,counterPoint); //Full U and V --> too much memory needed
	MatrixXd U(counterPoint,3); //ThinU and V
	MatrixXd S(3,1);
	MatrixXd V(3,3);

	JacobiSVD<MatrixXd> svd(m_translated, ComputeThinU | ComputeThinV);
	//JacobiSVD<MatrixXd> svd(m_translated, ComputeFullU | ComputeFullV);
	//NOTES:
	//[U,S,V]=SVD(M) where: M=nxm matrix
	//Then: U=nxn; S=nxm; V=mxm, for "ComputeThin" --> U=nxm
	U=svd.matrixU();
	S=svd.singularValues();
	V=svd.matrixV();

	//selecting the plane normal vector, which is the eigen vector correspond to smalles eigen value
	double singleVal=0.0;
	*n1=V(0,0);
	*n2=V(1,0);
	*n3=V(2,0);
	singleVal=S(0,0);
	for(i=1;i<=2;i++){
		if(singleVal>=S(i,0)){
			singleVal=S(i,0);
			*n1=V(0,i);
			*n2=V(1,i);
			*n3=V(2,i);
		}
	}

	*px=avgX;
	*py=avgY;
	*pz=avgZ;
	
	/*printf("\n\nThe matrix: \n");
	cout<<m_translated;
	printf("\n\nU: \n");
	cout<<U;
	printf("\n\nS: \n");
	cout<<S;
	printf("\n\nV: \n");
	cout<<V;*/
	
}

void myFunction::line_fitting(MatrixXd m, double *px, double *py, double *pz, double *n1, double *n2, double *n3){
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;

	counterPoint=m.rows();

	//calculating the points centroid	
	for(i=0;i<counterPoint;i++){
		xi=m(i,0);
		yi=m(i,1);
		zi=m(i,2);			
		sumX=sumX+xi;
		sumY=sumY+yi;
		sumZ=sumZ+zi;
	}
	avgX=sumX/counterPoint;
	avgY=sumY/counterPoint;
	avgZ=sumZ/counterPoint;

	//Transforming the points to their centroid
	MatrixXd m_hom(counterPoint,4);
	for(i=0;i<counterPoint;i++){
		m_hom(i,0)=m(i,0);
		m_hom(i,1)=m(i,1);
		m_hom(i,2)=m(i,2);
		m_hom(i,3)=1;
	}
	MatrixXd m_hom_trans(4,counterPoint);
	m_hom_trans=m_hom.transpose();
	MatrixXd m_hom_trans2(4,counterPoint);
	
	MatrixXd T(4,4);
	setTranslationMatrix(T,-avgX, -avgY,-avgZ);
		
	m_hom_trans2=T*m_hom_trans;

	MatrixXd m_translated(counterPoint,3);
	for(i=0;i<counterPoint;i++){
		m_translated(i,0)=m_hom_trans2(0,i);
		m_translated(i,1)=m_hom_trans2(1,i);
		m_translated(i,2)=m_hom_trans2(2,i);		
	}

	//SVD calculation of a rectangular matrix
	//MatrixXd U(counterPoint,counterPoint); //Full U and V --> too much memory needed
	MatrixXd U(counterPoint,3); //ThinU and V
	MatrixXd S(3,1);
	MatrixXd V(3,3);

	JacobiSVD<MatrixXd> svd(m_translated, ComputeThinU | ComputeThinV);
	//JacobiSVD<MatrixXd> svd(m_translated, ComputeFullU | ComputeFullV);
	//NOTES:
	//[U,S,V]=SVD(M) where: M=nxm matrix
	//Then: U=nxn; S=nxm; V=mxm, for "ComputeThin" --> U=nxm
	U=svd.matrixU();
	S=svd.singularValues();
	V=svd.matrixV();

	//selecting the plane normal vector, which is the eigen vector correspond to smalles eigen value
	double singleVal=0.0;
	*n1=V(0,0);
	*n2=V(1,0);
	*n3=V(2,0);
	singleVal=S(0,0);
	for(i=1;i<=2;i++){
		if(singleVal<=S(i,0)){
			singleVal=S(i,0);
			*n1=V(0,i);
			*n2=V(1,i);
			*n3=V(2,i);
		}
	}

	*px=avgX;
	*py=avgY;
	*pz=avgZ;		
}

void myFunction::sphere_fitting(MatrixXd m, double *px, double *py, double *pz, double *r, double *residual){
/*The algorithm is based on Lavenberg-Marquardt (LM) Algorithm*/
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;
	double p0[4],p[4], p_opt[4];

	//setting the initial valuf of the minimum and maximum (critical definition)
	double maxX=-9999999.9, maxY=-9999999.9, maxZ=-9999999.9;
	double minX=9999999.9, minY=9999999.9, minZ=9999999.9;

	counterPoint=m.rows();

	//calculating the points centroid	
	for(i=0;i<counterPoint;i++){
		xi=m(i,0);
		yi=m(i,1);
		zi=m(i,2);			
		sumX=sumX+xi;
		sumY=sumY+yi;
		sumZ=sumZ+zi;

		//Storing the maximum and minimum value
		if(maxX<xi){
			maxX=xi;
		}
		if(maxY<yi){
			maxY=yi;
		}
		if(maxZ<zi){
			maxZ=zi;
		}
		if(minX>xi){
			minX=xi;
		}
		if(minY>yi){
			minY=yi;
		}
		if(minZ>zi){
			minZ=zi;
		}

	}
	avgX=sumX/counterPoint;
	avgY=sumY/counterPoint;
	avgZ=sumZ/counterPoint;

	p0[3]=(((maxX-minX)/2)+((maxY-minY)/2))/2;	
	p0[0]=avgX;
	p0[1]=avgY;
	//p0[2]=avgZ;
	p0[2]=minZ;

	/*//--------------------------------
	//for special fitting condition for performance verification
	p0[0]=avgX;
	p0[1]=avgY;
	//p0[2]=maxZ-0.0025;
	//p0[3]=0.0025;
	p0[2]=maxZ-2500;
	p0[3]=2500;
	//---------------------------------*/
	
	//chaos_initial_point_sphere(m,&p0[0],&p0[1],&p0[2],&p0[3]);	
		
	double lamda=0.0001;
	double j0=0.0, j=0.0;
	VectorXd d(counterPoint);
	VectorXd x(4);
	MatrixXd F0(counterPoint,4);
	MatrixXd U(4,4);
	MatrixXd V(4,1);
	MatrixXd H(4,4);
	MatrixXd m_ident(4,4);
	MatrixXd m_diag(4,4);
	int counter1=0;
	int stop1=0, stop2=0, counter2=0;
	while(!stop1){
		p_opt[0]=p0[0];
		p_opt[1]=p0[1];
		p_opt[2]=p0[2];
		p_opt[3]=p0[3];
		lamda=lamda-0.04;
		//Build distance vector d
		for(i=0;i<counterPoint;i++){
			d(i)=sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]))-p0[3];
		}

		//Build F0 Matrix
		for(i=0;i<counterPoint;i++){
			F0(i,0)=-(m(i,0)-p0[0])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,1)=-(m(i,1)-p0[1])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,2)=-(m(i,2)-p0[2])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,3)=-1;
		}
		//calculate matrix U mxm (in this case 4x4) = F0'F0 where m=4 (x,y,z,r)
		U=F0.transpose()*F0;

		//calculate vector v mx1 (in this case 4x1)=F0'*d where m=4 (x,y,z,r)
		V=F0.transpose()*d;

		//calculate J0=sum (d^2) --> sum of square
		j0=0.0;
		for(i=0;i<counterPoint;i++){
			j0=j0+d(i)*d(i);
		}

		stop2=0;
		counter2=0;
		while(!stop2){
			lamda=lamda+10;
			//Define H matrix = Hessian Matrix.
			m_ident.setIdentity(4,4);
			m_diag.setZero(4,4);
			m_diag(0,0)=U(0,0);
			m_diag(1,1)=U(1,1);
			m_diag(2,2)=U(2,2);
			m_diag(3,3)=U(3,3);
			H=U+lamda*(m_ident+m_diag);

			//%solving Hx=-v --> LSQ equation
			x=H.lu().solve(-1*V);

			//Update the P new
			p[0]=p0[0]+x(0);
			p[1]=p0[1]+x(1);
			p[2]=p0[2]+x(2);
			p[3]=p0[3]+x(3);

			//Recalculate the distance with new P and Recalculate the new j (sum of square)
			for(i=0;i<counterPoint;i++){
				d(i)=sqrt((m(i,0)-p[0])*(m(i,0)-p[0])+(m(i,1)-p[1])*(m(i,1)-p[1])+(m(i,2)-p[2])*(m(i,2)-p[2]))-p[3];
			}

			j=0.0;
			for(i=0;i<counterPoint;i++){
				j=j+d(i)*d(i);
			}

			if((p[0]==p0[0]) && (p[1]==p0[1]) && (p[2]==p0[2]) && (p[3]==p0[3])){// wif P = P0, then it converges
				p0[0]=p[0];
				p0[1]=p[1];
				p0[2]=p[2];
				p0[3]=p[3];

				p_opt[0]=p0[0];
				p_opt[1]=p0[1];
				p_opt[2]=p0[2];
				p_opt[3]=p0[3];

				*px=p_opt[0];
				*py=p_opt[1];
				*pz=p_opt[2];
				*r=p_opt[3];

				return;
			}
			
			if(j<j0){
				stop2=1;
			}
			counter2=counter2+1;
			if(counter2>20){
				stop2=1;
			}
		}

		if(j<j0){
			p0[0]=p[0];
			p0[1]=p[1];
			p0[2]=p[2];
			p0[3]=p[3];			
		}
		counter1=counter1+1;
		if(counter1>20){
			stop1=1;
		}
	}

	*px=p_opt[0];
	*py=p_opt[1];
	*pz=p_opt[2];
	*r=p_opt[3];
	*residual=j;
	
}

void myFunction::sphere_fitting_25_points(MatrixXd m_all, MatrixXd m, double *px, double *py, double *pz, double *r){//Lavenberg-Marquardt (LM) Algorithm
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;
	double maxX=-99999999.9, maxY=-99999999.9, maxZ=-99999999.9;
	double minX=99999999.9, minY=99999999.9, minZ=99999999.9;
	double p0[4],p[4], p_opt[4];

	counterPoint=m_all.rows();

	//calculating the points centroid	
	for(i=0;i<counterPoint;i++){
		xi=m_all(i,0);
		yi=m_all(i,1);
		zi=m_all(i,2);			
		sumX=sumX+xi;
		sumY=sumY+yi;
		sumZ=sumZ+zi;

		//Storing the maximum and minimum value
		if(maxX<xi){
			maxX=xi;
		}
		if(maxY<yi){
			maxY=yi;
		}
		if(maxZ<zi){
			maxZ=zi;
		}
		if(minX>xi){
			minX=xi;
		}
		if(minY>yi){
			minY=yi;
		}
		if(minZ>zi){
			minZ=zi;
		}

	}
	avgX=sumX/counterPoint;
	avgY=sumY/counterPoint;
	avgZ=sumZ/counterPoint;

	p0[3]=(((maxX-minX)/2)+((maxY-minY)/2))/2;	
	p0[0]=avgX;
	p0[1]=avgY;
	//p0[2]=avgZ;
	p0[2]=minZ;
	
	/* for special purpose: 4-axes performance verification sphere fitting
	//----------for 1st + rotation----------------------
	//p0[0]=avgX;
	//p0[1]=avgY;
	//p0[2]=maxZ-2500;
	//p0[3]=2500;
	//---------------------------------

	//----------for 2nd + rotation----------------------
	p0[0]=avgX;	
	p0[1]=minY+2500;
	p0[2]=avgZ;
	p0[3]=2500;

	//for tweaking
	//p0[0]=avgX;	
	//p0[1]=minY+1000;
	//p0[2]=avgZ+1000;
	//p0[3]=2500;
	//---------------------------------

	//----------for 3rd + rotation----------------------
	//p0[0]=avgX;
	//p0[1]=avgY;
	//p0[2]=minZ+2500;
	//p0[3]=2500;
	//---------------------------------

	//----------for 4th + rotation----------------------
	//p0[0]=avgX;
	//p0[1]=maxY-2500;	
	//p0[2]=avgZ;
	//p0[3]=2500;
	//--------------------------------------------------*/
	
	counterPoint=0;
	counterPoint=m.rows();

	//chaos_initial_point_sphere(m,&p0[0],&p0[1],&p0[2],&p0[3]);		
	
	double lamda=0.0001;
	double j0=0.0, j=0.0;
	VectorXd d(counterPoint);
	VectorXd x(4);
	MatrixXd F0(counterPoint,4);
	MatrixXd U(4,4);
	MatrixXd V(4,1);
	MatrixXd H(4,4);
	MatrixXd m_ident(4,4);
	MatrixXd m_diag(4,4);
	int counter1=0;
	int stop1=0, stop2=0, counter2=0;
	while(!stop1){
		p_opt[0]=p0[0];
		p_opt[1]=p0[1];
		p_opt[2]=p0[2];
		p_opt[3]=p0[3];
		lamda=lamda-0.04;
		//Build distance vector d
		for(i=0;i<counterPoint;i++){
			d(i)=sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]))-p0[3];
		}

		//Build F0 Matrix
		for(i=0;i<counterPoint;i++){
			F0(i,0)=-(m(i,0)-p0[0])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,1)=-(m(i,1)-p0[1])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,2)=-(m(i,2)-p0[2])/sqrt((m(i,0)-p0[0])*(m(i,0)-p0[0])+(m(i,1)-p0[1])*(m(i,1)-p0[1])+(m(i,2)-p0[2])*(m(i,2)-p0[2]));
			F0(i,3)=-1;
		}
		//calculate matrix U mxm (in this case 4x4) = F0'F0 where m=4 (x,y,z,r)
		U=F0.transpose()*F0;

		//calculate vector v mx1 (in this case 4x1)=F0'*d where m=4 (x,y,z,r)
		V=F0.transpose()*d;

		//calculate J0=sum (d^2) --> sum of square
		j0=0.0;
		for(i=0;i<counterPoint;i++){
			j0=j0+d(i)*d(i);
		}

		stop2=0;
		counter2=0;
		while(!stop2){
			lamda=lamda+10;
			//Define H matrix = Hessian Matrix.
			m_ident.setIdentity(4,4);
			m_diag.setZero(4,4);
			m_diag(0,0)=U(0,0);
			m_diag(1,1)=U(1,1);
			m_diag(2,2)=U(2,2);
			m_diag(3,3)=U(3,3);
			H=U+lamda*(m_ident+m_diag);

			//%solving Hx=-v --> LSQ equation
			x=H.lu().solve(-1*V);

			//Update the P new
			p[0]=p0[0]+x(0);
			p[1]=p0[1]+x(1);
			p[2]=p0[2]+x(2);
			p[3]=p0[3]+x(3);

			//Recalculate the distance with new P and Recalculate the new j (sum of square)
			for(i=0;i<counterPoint;i++){
				d(i)=sqrt((m(i,0)-p[0])*(m(i,0)-p[0])+(m(i,1)-p[1])*(m(i,1)-p[1])+(m(i,2)-p[2])*(m(i,2)-p[2]))-p[3];
			}

			j=0.0;
			for(i=0;i<counterPoint;i++){
				j=j+d(i)*d(i);
			}

			if((p[0]==p0[0]) && (p[1]==p0[1]) && (p[2]==p0[2]) && (p[3]==p0[3])){// wif P = P0, then it converges
				p0[0]=p[0];
				p0[1]=p[1];
				p0[2]=p[2];
				p0[3]=p[3];

				p_opt[0]=p0[0];
				p_opt[1]=p0[1];
				p_opt[2]=p0[2];
				p_opt[3]=p0[3];

				*px=p_opt[0];
				*py=p_opt[1];
				*pz=p_opt[2];
				*r=p_opt[3];

				return;
			}
			
			if(j<j0){
			//if(j>j0){
				stop2=1;
			}
			counter2=counter2+1;
			if(counter2>1000){
				stop2=1;
			}
		}

		if(j<j0){
		//if(j>j0){
			p0[0]=p[0];
			p0[1]=p[1];
			p0[2]=p[2];
			p0[3]=p[3];			
		}
		counter1=counter1+1;
		if(counter1>1000){
			stop1=1;
		}
	}

	*px=p_opt[0];
	*py=p_opt[1];
	*pz=p_opt[2];
	*r=p_opt[3];	
}

void myFunction::cylinder_fitting(MatrixXd m, double *px, double *py, double *pz, double *a1,double *a2,double *a3, double *r, double* residual){//Lavenberg-Marquardt (LM) Algorithm
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;
	double maxX=0.0, maxY=0.0, maxZ=0.0;
	double minX=0.0, minY=0.0, minZ=0.0;
	double p0[7],p[7], p_opt[7];

	counterPoint=m.rows();

	/*//calculating the points centroid	
	for(i=0;i<counterPoint;i++){
		xi=m(i,0);
		yi=m(i,1);
		zi=m(i,2);			
		sumX=sumX+xi;
		sumY=sumY+yi;
		sumZ=sumZ+zi;

		//Storing the maximum and minimum value
		if(maxX<xi){
			maxX=xi;
		}
		if(maxY<yi){
			maxY=yi;
		}
		if(maxZ<zi){
			maxZ=zi;
		}
		if(minX>xi){
			minX=xi;
		}
		if(minY>yi){
			minY=yi;
		}
		if(minZ>zi){
			minZ=zi;
		}

	}
	avgX=sumX/counterPoint;
	avgY=sumY/counterPoint;
	avgZ=sumZ/counterPoint;
	
	p0[6]=(((maxX-minX)/2)+((maxY-minY)/2))/2;	
	p0[0]=avgX;
	p0[1]=avgY;
	p0[2]=avgZ;	*/	

	//Initial point
	line_fitting(m,&p0[0],&p0[1],&p0[2],&p0[3],&p0[4],&p0[5]); //as initial estimate of px,py,pz,a1,a2,a3
	p0[6]=dist_3D_point_to_line(m(1,0),m(1,1),m(1,2),p0[0],p0[1],p0[2],p0[3],p0[4],p0[5]);//just a distance from a selected point to the initial axis line

	/*p0[0]=*px;
	p0[1]=*py;
	p0[2]=*pz;
	p0[3]=*a1;
	p0[4]=*a2;
	p0[5]=*a3;
	p0[6]=*r;*/

	//--------------------------------	
		
	//chaos_initial_point_sphere(m,&p0[0],&p0[1],&p0[2],&p0[3]);	
		
	double lamda=0.0001;
	double lengthA=0.0;
	double a[3];
	double uSquare=0.0;
	double vSquare=0.0;
	double wSquare=0.0;
	double j0=0.0, j=0.0;
	VectorXd d(counterPoint);
	VectorXd x(7);
	VectorXd g(counterPoint);
	VectorXd f(counterPoint);
	MatrixXd F0(counterPoint,7);
	MatrixXd U(7,7);
	MatrixXd V(7,1);
	MatrixXd H(7,7);
	MatrixXd m_ident(7,7);
	MatrixXd m_diag(7,7);
	int counter1=0;
	int stop1=0, stop2=0, counter2=0;
	while(!stop1){
		p_opt[0]=p0[0];
		p_opt[1]=p0[1];
		p_opt[2]=p0[2];
		p_opt[3]=p0[3];
		p_opt[4]=p0[4];
		p_opt[5]=p0[5];
		p_opt[6]=p0[6];
		lamda=lamda-0.04;

		//calculate vector a=A/|A|
		lengthA=sqrt(p0[3]*p0[3]+p0[4]*p0[4]+p0[5]*p0[5]);
		a[0]=p0[3]/lengthA; //a
		a[1]=p0[4]/lengthA; //b
		a[2]=p0[5]/lengthA; //c

		j0=0.0;
		for(i=0;i<counterPoint;i++){
			//calculate gi=a.(xi-x) --> scalar
			g(i)=a[0]*(m(i,0)-p0[0])+a[1]*(m(i,1)-p0[1])+a[2]*(m(i,2)-p0[2]);

			//calculate fi=sqrt(u^2+v^2+w^2) --> scalar
			uSquare=(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]))*(a[2]*(m(i,1)-p0[1])-a[1]*(m(i,2)-p0[2]));
			vSquare=(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]))*(a[0]*(m(i,2)-p0[2])-a[2]*(m(i,0)-p0[0]));
			wSquare=(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]))*(a[1]*(m(i,0)-p0[0])-a[0]*(m(i,1)-p0[1]));
			f(i)=sqrt(uSquare+vSquare+wSquare);

			//Build distance vector d
			d(i)=f(i)-p0[6];
			//d(i)=fabs(f(i)-p0[6]);

			//calculate J0=sum (d^2) --> sum of square
			j0=j0+d(i)*d(i);
	
			//Build F0 Matrix
			if(f(i)!=0){
				F0(i,0)=(a[0]*g(i)-(m(i,0)-p0[0]))/f(i);
				F0(i,1)=(a[1]*g(i)-(m(i,1)-p0[1]))/f(i);
				F0(i,2)=(a[2]*g(i)-(m(i,2)-p0[2]))/f(i);
				F0(i,3)=g(i)*(a[0]*g(i)-(m(i,0)-p0[0]))/f(i);;
				F0(i,4)=g(i)*(a[1]*g(i)-(m(i,1)-p0[1]))/f(i);
				F0(i,5)=g(i)*(a[2]*g(i)-(m(i,2)-p0[2]))/f(i);
			}
			else{ //f(i)=0
				F0(i,0)=sqrt(1-a[0]*a[0]);
				F0(i,1)=sqrt(1-a[1]*a[1]);
				F0(i,2)=sqrt(1-a[2]*a[2]);
				F0(i,3)=g(i)*sqrt(1-a[0]*a[0]);
				F0(i,4)=g(i)*sqrt(1-a[1]*a[1]);
				F0(i,5)=g(i)*sqrt(1-a[2]*a[2]);
			}
			F0(i,6)=-1;
		}
		//calculate matrix U mxm (in this case 4x4) = F0'F0 where m=4 (x,y,z,r)
		U=F0.transpose()*F0;

		//calculate vector v mx1 (in this case 4x1)=F0'*d where m=4 (x,y,z,r)
		V=F0.transpose()*d;

		//calculate J0=sum (d^2) --> sum of square
		/*j0=0.0;
		for(i=0;i<counterPoint;i++){
			j0=j0+d(i)*d(i);
		}*/

		stop2=0;
		counter2=0;
		while(!stop2){
			lamda=lamda+10;
			//Define H matrix = Hessian Matrix.
			m_ident.setIdentity(7,7);
			m_diag.setZero(7,7);
			m_diag(0,0)=U(0,0);
			m_diag(1,1)=U(1,1);
			m_diag(2,2)=U(2,2);
			m_diag(3,3)=U(3,3);
			m_diag(4,4)=U(4,4);
			m_diag(5,5)=U(5,5);
			m_diag(6,6)=U(6,6);
			H=U+lamda*(m_ident+m_diag);

			//%solving Hx=-v --> LSQ equation
			x=H.lu().solve(-1*V);

			//Update the P new
			p[0]=p0[0]+x(0);
			p[1]=p0[1]+x(1);
			p[2]=p0[2]+x(2);
			p[3]=p0[3]+x(3);
			p[4]=p0[3]+x(4);
			p[5]=p0[3]+x(5);
			p[6]=p0[3]+x(6);

			//Recalculate the distance with new P and Recalculate the new j (sum of square)
			j=0.0;
			for(i=0;i<counterPoint;i++){
				//calculate gi=a.(xi-x) --> scalar
				g(i)=a[0]*(m(i,0)-p[0])+a[1]*(m(i,1)-p[1])+a[2]*(m(i,2)-p[2]);

				//calculate fi=sqrt(u^2+v^2+w^2) --> scalar
				uSquare=(a[2]*(m(i,1)-p[1])-a[1]*(m(i,2)-p[2]))*(a[2]*(m(i,1)-p[1])-a[1]*(m(i,2)-p[2]));
				vSquare=(a[0]*(m(i,2)-p[2])-a[2]*(m(i,0)-p[0]))*(a[0]*(m(i,2)-p[2])-a[2]*(m(i,0)-p[0]));
				wSquare=(a[1]*(m(i,0)-p[0])-a[0]*(m(i,1)-p[1]))*(a[1]*(m(i,0)-p[0])-a[0]*(m(i,1)-p[1]));
				f(i)=sqrt(uSquare+vSquare+wSquare);

				//Build distance vector d
				d(i)=f(i)-p[6];
				//d(i)=fabs(f(i)-p[6]);

				//j sum
				j=j+d(i)*d(i);
			}

			/*j=0.0;
			for(i=0;i<counterPoint;i++){
				j=j+d(i)*d(i);
			}*/

			if((p[0]==p0[0]) && (p[1]==p0[1]) && (p[2]==p0[2]) && (p[3]==p0[3]) && (p[4]==p0[4]) && (p[5]==p0[5]) && (p[6]==p0[6])){// wif P = P0, then it converges
				p0[0]=p[0];
				p0[1]=p[1];
				p0[2]=p[2];
				p0[3]=p[3];
				p0[4]=p[4];
				p0[5]=p[5];
				p0[6]=p[6];

				p_opt[0]=p0[0];
				p_opt[1]=p0[1];
				p_opt[2]=p0[2];
				p_opt[3]=p0[3];
				p_opt[4]=p0[4];
				p_opt[5]=p0[5];
				p_opt[6]=p0[6];

				*px=p_opt[0];
				*py=p_opt[1];
				*pz=p_opt[2];
				*a1=p_opt[3];
				*a2=p_opt[4];
				*a3=p_opt[5];
				*r=p_opt[6];

				return;
			}
			
			if(j<j0){
				stop2=1;
			}
			counter2=counter2+1;
			if(counter2>100){
				stop2=1;
			}
		}

		if(j<j0){
			p0[0]=p[0];
			p0[1]=p[1];
			p0[2]=p[2];
			p0[3]=p[3];		
			p0[4]=p[4];	
			p0[5]=p[5];	
			p0[6]=p[6];	
		}
		counter1=counter1+1;
		if(counter1>50){
			stop1=1;
		}
	}

	*px=p_opt[0];
	*py=p_opt[1];
	*pz=p_opt[2];
	*a1=p_opt[3];
	*a2=p_opt[4];
	*a3=p_opt[5];
	*r=p_opt[6];
	*residual=j;
	
}

//========================== FILTERING ALGORITMH =========================================
/*
NOTE: The filtering is based on ISO 16610 series, especially for areal (2D) fultering.
*/
//void calculate_row_col_zAvg(MatrixXd m, int *nrow, int *ncol, double *zAvg1){
MatrixXd myFunction::calculate_row_col_zAvg(MatrixXd m, int *nrow, int *ncol, double *zAvg1){
	//NOTE: The goal of the function is to determine the num or row and column and zAvg to close a hole
	// the return value of the function is a matrix containing the list of y coordinate

	//Calculate: Average z to replace a hole in the data set
	int o;
	double zSum=0;
	for(o=0;o<m.rows();o++){
		zSum=zSum+m(o,2);
	}
	double zAvg=zSum/(m.rows());

	//Calculate number of column in the first row
	double yCurr=0.0;
	double yPrev=0.0;
	int flag=1;
	int countNum=0;
	while(flag){
		if(countNum==0){			
			yCurr=m(0,1); //Initialization of the Y value. The sama Y means that the same row
			yPrev=yCurr;
			countNum++;
		}
		else{
			yCurr=m(countNum,1);
			if(fabs(yPrev-yCurr)<0.0000000001){ //still at the same row
				countNum++;
				//yPrev=yCurr;
			}
			else{
				flag=0;
			}
		}

	}

	//Calculate number of row
	
	double xCurr=0.0;
	double xPrev=0.0;
	flag=1;
	int countNum2=0;
	for(o=0;o<m.rows();o++){
		if(countNum2==0){
			xCurr=m(0,0); //Initialization of the X value. The sama X means that the same column
			xPrev=xCurr;
			countNum2++;
		}
		else{
			xCurr=m(o,0);
			if(fabs(xPrev-xCurr)<0.00000001){ //they are in the same column
				countNum2++;
			}			
		}
	}

	*nrow=countNum2;
	*ncol=countNum;
	*zAvg1=zAvg;

	MatrixXd yCoord(countNum2,1);
	int indexY=0;
	for(o=0;o<m.rows();o++){
		if(indexY==0){
			xCurr=m(0,0); //Initialization of the X value. The sama X means that the same column
			xPrev=xCurr;
			yCoord(indexY,0)=m(0,1);
			indexY++;
		}
		else{
			xCurr=m(o,0);
			if(fabs(xPrev-xCurr)<0.00000001){ //they are in the same column
				yCoord(indexY,0)=m(o,1);
				indexY++;
			}			
		}
	}
	
	return yCoord;
}	

MatrixXd myFunction::linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
//void linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
	/*
	NOTE:
	deltaX=2.6 um
	deltaY=2.6 um
	image matrix size = 1624x1232
	first we have to now number of x and y since the matrix size reduces due to alicona decimation
	*/
	
	int num_of_column=0, num_of_row=0;
	double zAvg=0.0;
	int i=0, j=0;
	double yVal=0.0, xVal=0.0;
	
	//STEP 1: Calculating zAvg to close hole, num of row and col
	MatrixXd yCoord;
	calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	yCoord=calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	//cout<<yCoord;
	printf("\nNUMBER OF COLUMN= %d\n",num_of_column);
	printf("\nNUMBER OF ROW= %d\n",num_of_row);
	printf("\nAVERAGE Z= %f\n",zAvg);

	//STEP 2: Reconstructing the 2D matrix for 3D MAP.
	MatrixXd mx2D(num_of_row,num_of_column);
	MatrixXd my2D(num_of_row,num_of_column);
	MatrixXd mz2D(num_of_row,num_of_column);
	MatrixXd mz2D_temp(num_of_row,num_of_column);
	MatrixXd mz2D_filtered(num_of_row,num_of_column);
	int indexCount=0;	
	yVal=0.0;
	for(j=0;j<num_of_row;j++){ //filling the matrix is in row by row fashion (column scanning)
		yVal=yCoord(j,0);
		for(i=0;i<num_of_column;i++){
			if(fabs(yVal-m(indexCount,1))<0.000000001){
				mz2D(j,i)=m(indexCount,2); //NOTE: we only save the Z position since
															//the x and y position are in index form					
			}
			else{
				mz2D(j,i)=zAvg;
				//if there is a hole, do not increase the hole counting
			}
			mx2D(j,i)=m(indexCount,0); 
			my2D(j,i)=m(indexCount,1); 
			indexCount++;
		}
	}
	//cout<<"\n\nm_x:\n"<<mx2D<<"\n\n";
	//cout<<"\n\nm_y:\n"<<my2D<<"\n\n";
	//cout<<"\n\nm_z:\n"<<mz2D<<"\n\n";

	//STEP 3: Linear Gaussian Filter: ISO16610-61
	//NOTE: only processing matrix mz2D
	const double alfa=0.44697; //from ISO 16610-61
	const double du=2.6, dv=2.6;
	const int nWindow=lamda_c/du;	
	//const int nWindow=3; //for testing
	
	double sum_numerator=0.0;
	double sum_denumerator=0.0;
	double power=0.0;
	const double e=2.718281828;
	const double phi=3.1410320;

	int k=0;
	double val1=0.0, val2=0.0;

	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){ //Filtering in X-direction (column-wise)
			sum_numerator=0.0;
			sum_denumerator=0.0;
			for(k=-nWindow/2;k<nWindow/2;k++){
				power=-1*phi*((i-k)/(alfa*lamda_c))*((i-k)/(alfa*lamda_c));
				if((i-nWindow/2>=0) && (i+nWindow/2<=num_of_column-1)){ //if the range is lamda_c<n.du<lt-lamda_c (inside the 3D map matrix)
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;					
				}				
				else if((i-k < 0) || (i-k > num_of_column-1)){							
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i+k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}
				else if((i+k < 0) || (i+k > num_of_column-1)){						
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}		
				else{
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}
				sum_numerator=sum_numerator+val1;
				sum_denumerator=sum_denumerator+val2;
			}
			if(sum_denumerator!=0){
				mz2D_temp(j,i)=sum_numerator/sum_denumerator;
			}
			else{
				mz2D_temp(j,i)=mz2D(j,i);
			}
		}
	}
	for(i=0;i<num_of_column;i++){
		for(j=0;j<num_of_row;j++){ //Filtering in Y-direction (row-wise)
			sum_numerator=0.0;
			sum_denumerator=0.0;
			for(k=-nWindow/2;k<nWindow/2;k++){
				power=-1*phi*((j-k)/(alfa*lamda_c))*((j-k)/(alfa*lamda_c));
				if((j-nWindow/2>=0) && (j+nWindow/2<=num_of_row-1)){ //if the range is lamda_c<n.du<lt-lamda_c (inside the 3D map matrix)
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;					
				}				
				else if((j-k < 0) || (j-k > num_of_row-1)){							
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j+k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;	
				}
				else if((j+k < 0) || (j+k > num_of_row-1)){					
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;
				}						
				else{
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;	
				}
				sum_numerator=sum_numerator+val1;
				sum_denumerator=sum_denumerator+val2;				
			}
			if(sum_denumerator!=0){
				mz2D_filtered(j,i)=sum_numerator/sum_denumerator;
			}
			else{
				mz2D_filtered(j,i)=mz2D_temp(j,i);
			}
		
		}
	}
	//cout<<mz2D_filtered;
	
	//STEP 4: Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}
	//MatrixXd m_filtered(1,1);
	//m_filtered(0,0)=1;//just for test
	return m_filtered;
	
}

MatrixXd myFunction::robust_gaussian_regression_filter(MatrixXd m, double lamda_c){ //return the filtered matrix

	/*
	NOTE:
	deltaX=2.6 um
	deltaY=2.6 um
	image matrix size = 1624x1232
	first we have to now number of x and y since the matrix size reduces due to alicona decimation
	*/
	
	int num_of_column=0, num_of_row=0;
	double zAvg=0.0;
	int i=0, j=0, k=0, l=0;
	double yVal=0.0, xVal=0.0;
	
	//STEP 1: Calculating zAvg to close hole, num of row and col
	MatrixXd yCoord;
	calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	yCoord=calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	//cout<<yCoord;
	printf("\nNUMBER OF COLUMN= %d\n",num_of_column);
	printf("\nNUMBER OF ROW= %d\n",num_of_row);
	printf("\nAVERAGE Z= %f\n",zAvg);

	//STEP 2: Reconstructing the 2D matrix for 3D MAP.
	MatrixXd mx2D(num_of_row,num_of_column);
	MatrixXd my2D(num_of_row,num_of_column);
	MatrixXd mz2D(num_of_row,num_of_column);	
	int indexCount=0;	
	yVal=0.0;
	for(j=0;j<num_of_row;j++){ //filling the matrix is in row by row fashion (column scanning)
		yVal=yCoord(j,0);
		for(i=0;i<num_of_column;i++){
			if(fabs(yVal-m(indexCount,1))<0.000000001){
				mz2D(j,i)=m(indexCount,2); //NOTE: we only save the Z position since
															//the x and y position are in index form					
			}
			else{
				mz2D(j,i)=zAvg;
				//if there is a hole, do not increase the hole counting
			}
			mx2D(j,i)=m(indexCount,0); 
			my2D(j,i)=m(indexCount,1); 
			indexCount++;
		}
	}

	//STEP 3: Robust Gaussian Regression Filter: ISO16610-71
	const double alfa=0.44697; //from ISO 16610-61
	const double gamma=0.7309; //from ISO 16610-71
	//const double du=2.6, dv=2.6;
	//const int nWindow=lamda_c/du;	
	const int nWindow=3; //for testing
	const double du=1, dv=1; //for testing

	const double e=2.718281828;
	const double phi=3.1410320;

	MatrixXd mz2D_filtered(num_of_row,num_of_column);
	MatrixXd Xij(nWindow*nWindow,6);
	MatrixXd x_ij(nWindow,nWindow);
	MatrixXd y_ij(nWindow,nWindow);
	MatrixXd Xij_trans(6,nWindow*nWindow);
	MatrixXd Sij(nWindow*nWindow,nWindow*nWindow);
	MatrixXd z(nWindow*nWindow,1);
	MatrixXd mat(1,6);
	mat<<1,0,0,0,0,0; //matrix initialization
	MatrixXd wij(1,1); //to store the wij value as result of matrix calculation
	MatrixXd XSX(6,6);
	MatrixXd XSX_inv(6,6);

	int k_row=0, k_col=0, index_row=0, index_col=0, index_z=0;
	int index_i=0, index_j=0;
	int k_counter=0, l_counter=0;
	double wijAvg=0.0, wij_sum=0.0;
	indexCount=0;
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			//STEP 3.1: Building matrix z, x_ij, and y_ij	
			wij_sum=0.0;
			k_counter=0;
			l_counter=0;
			index_z=0;
			index_col=-1*nWindow/2;
			for(k_col=0;k_col<nWindow;k_col++){ //row-wise
				l_counter++;
				index_row=-1*nWindow/2;
				for(k_row=0;k_row<nWindow;k_row++){
					k_counter++;
					//Determining the row-index
					//if((j+index_row)>0 && (j+index_row)<num_of_row-1){ // use j+index_row
					//}
					if((j+index_row)<0 || (j+index_row)>num_of_row-1){ // use j-index_row
						index_i=j-index_row;
					}
					else{ // use j+index_row
						index_i=j+index_row;
					}
			
					//Determining the column-index
					if((i+index_col)<0 || (i+index_col)>num_of_column-1){ // use i-index_row
						index_j=i-index_col;
					}
					else{ // use i+index_row
						index_j=i+index_col;
					}
					
					x_ij(k_row,k_col)=index_row*du;
					y_ij(k_row,k_col)=index_col*dv;
					//x_ij(k_row,k_col)=(k_counter-j)*du;
					//y_ij(k_row,k_col)=(l_counter-1)*dv;
					z(index_z)=mz2D(index_i,index_j);
					wij_sum=wij_sum+mz2D(index_i,index_j);
					
					index_row++;
					index_z++;
				}
				index_col++;
			}
			wijAvg=wij_sum/(nWindow*nWindow);

			//STEP 3.2: Building matrix Xij and Xij Transpose
			indexCount=0;
			for(k=0;k<nWindow;k++){
				for(l=0;l<nWindow;l++){ //row fashion
					Xij(indexCount,0)=1;
					Xij(indexCount,1)=x_ij(l,k);
					Xij(indexCount,2)=y_ij(l,k);
					Xij(indexCount,3)=x_ij(l,k)*y_ij(l,k);
					Xij(indexCount,4)=x_ij(l,k)*x_ij(l,k);
					Xij(indexCount,5)=y_ij(l,k)*y_ij(l,k);
					indexCount++;
				}
			}

			Xij_trans=Xij.transpose();

			//STEP 3.3: Building matrix Sij
			Sij.setZero(nWindow*nWindow,nWindow*nWindow);
			double s_klij=0.0;
			double power=0.0;
			double delta_ij=0.0;
			double trisula=0.0;
			double c=0.0;
			const double e=2.718281828;
			const double gamma=0.7309;
			const double phi=3.1410320;
			for(k=0;k<nWindow*nWindow;k++){ //the problem is here in constructing the matrix S
				for(l=0;l<nWindow*nWindow;l++){ //row fashion
					if(k==l){
						//power=(-1*phi/(gamma*gamma))*((x_ij(l,k)*x_ij(l,k)+y_ij(l,k)*y_ij(l,k))/(gamma*gamma));
						power=(-1*phi/(gamma*gamma))*((x_ij(l%nWindow,k%nWindow)*x_ij(l%nWindow,k%nWindow)+y_ij(l%nWindow,k%nWindow)*y_ij(l%nWindow,k%nWindow))/(gamma*gamma));
						s_klij=(1/(gamma*gamma*lamda_c*lamda_c))*pow(e,power);

						c=(3*(mz2D(j,i)-wijAvg))/0.6745; //0.6745 calculated in MATLAB
						c=5; //in um //for testing
						if(fabs(mz2D(j,i)-wijAvg)<=c){
							if(c==0){//To avoid division by zero
								c=0.000000001;
							}
							trisula=(mz2D(j,i)-wijAvg)*(1-((mz2D(j,i)-wijAvg)/c)*((mz2D(j,i)-wijAvg)/c))*(1-((mz2D(j,i)-wijAvg)/c)*((mz2D(j,i)-wijAvg)/c));
						}
						else{
							trisula=0;
							//trisula=1;
						}
												
						if((mz2D(j,i)-wijAvg)!=0){ //to avoid division by zero
							delta_ij=trisula/(mz2D(j,i)-wijAvg); //mz2D(j,i) is the currect z on this point
						}
						else{
							delta_ij=1;//for testing
						}

						Sij(l,k)=s_klij*delta_ij;
						//cout<<"\nSij\n"<<Sij(l,k);
						if(Sij(l,k)==0){//To avoid singlar matrix of XSX matrix
							Sij(l,k)=0.0000000001;  
						}
						//Sij(l,k)=1;  //for testing
					}
				}
			}

			//STEP 3.4: Calculating matrix XSX
			XSX=Xij_trans*Sij*Xij;
			XSX_inv=XSX.inverse();

			//STEP 3.5: Calculating wij
			wij=mat*XSX_inv*Xij_trans*Sij*z;
			mz2D_filtered(j,i)=wij(0,0); //to access the calculated wij value
		}
	}
	
	//STEP 4: Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}
	//MatrixXd m_filtered(1,1);
	//m_filtered(0,0)=1;//just for test
	//cout<<"\n\nz\n"<<z; //OK
	//cout<<"\n\nx_ij\n"<<x_ij; //OK
	//cout<<"\n\ny_ij\n"<<y_ij; //OK
	//cout<<"\n\nXij\n"<<Xij;  //OK
	//cout<<"\n\nXij_trans\n"<<Xij_trans; //OK

	//cout<<"\n\nSij\n"<<Sij; //PROBLEM
	//cout<<"\n\nXSX\n"<<XSX; //PROBLEM
	//cout<<"\n\nXSX Inverse\n"<<XSX_inv; //PROBLEM
	//cout<<"\n\nm_ORIGINAL\n"<<m;
	//cout<<"\n\nm_FILTERED\n"<<m_filtered;

	/*MatrixXd test(3,3); //Test inverse matrix calculation OK
	test<<1,2,3,
		  9,5,7,
		  10,1,5;
	cout<<"\n\ntest inverse matrix\n"<<test.inverse()*test;*/

	return m_filtered;

}

MatrixXd myFunction::test_return_matrix(MatrixXd m,int X, int y){
	MatrixXd x(3,3);

	x(0,0)=1; x(0,1)=2; x(0,2)=3;
	x(1,0)=4; x(1,1)=5; x(1,2)=6;
	x(2,0)=7; x(2,1)=8; x(2,2)=9;

	return x;

}

//====================== OUTLIER CORRECTION FILTER ==================================================
double myFunction::find_threshold(VectorXd vect,int vector_length){
	double thresh=0.0;
	int i,j;
	double temp=0.0;

	//short the vector vect Ascending by Selection Sort
	for(i=0;i<vector_length-1;i++){
		temp=vect(i);
		for(j=i+1;j<vector_length;j++){
			if(vect(j)<temp){//change
				vect(i)=vect(j);
				vect(j)=temp;
				temp=vect(i);
			}
		}
	}

	int index=0;
	index=(int)0.99*vector_length;
	thresh=vect(index);
	return thresh;
}

double myFunction::find_median(VectorXd &Sij, int vector_length){// function to find a median from a vector of data set
	double median=0;
	int i=0, j=0, index_median=0;
	double temp=0.0;
	//short the vector Sij Ascending by Selection Sort
	for(i=0;i<vector_length-1;i++){
		temp=Sij(i);
		for(j=i+1;j<vector_length;j++){
			if(Sij(j)<temp){//change
				Sij(i)=Sij(j);
				Sij(j)=temp;
				temp=Sij(i);
			}
		}
	}
	index_median=(int)vector_length/2;
	median=Sij(index_median);
	return median;
}

double myFunction::median_calculation(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j,int grid_size){
	int k=0, l=0;
	double median=0;
	VectorXd Sij(grid_size*grid_size);
	int counter=0;
	for(k=-grid_size/2 ;k<=grid_size/2;k++){
		for(l=-grid_size/2;l<=grid_size/2;l++){
			Sij(counter)=fabs(mz2D(i,j)-mz2D(i+k,j+l));
			counter++;
		}
	}	
	median=find_median(Sij,counter);
	return median;	
}

double myFunction::median_calculation2(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j){
	int k=0, l=0;
	double median=0;
	VectorXd Sij(8); //calculating the relative distance on 8 neighbourhood
	int counter=0;

	Sij(0)=fabs(mz2D(i,j)-mz2D(i-1,j-1));
	Sij(1)=fabs(mz2D(i,j)-mz2D(i-1,j));
	Sij(2)=fabs(mz2D(i,j)-mz2D(i-1,j+1));
	Sij(3)=fabs(mz2D(i,j)-mz2D(i,j-1));
	Sij(4)=fabs(mz2D(i,j)-mz2D(i,j+1));
	Sij(5)=fabs(mz2D(i,j)-mz2D(i+1,j-1));
	Sij(6)=fabs(mz2D(i,j)-mz2D(i+1,j));
	Sij(7)=fabs(mz2D(i,j)-mz2D(i+1,j+1));
	
	median=find_median(Sij,8);
	return median;	
}

double myFunction::outlier_correction(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j,int grid_size, double T1, double T2, double zAvg){
	double filtered_value=0;
	int k=0, l=0;

	//STAGE 1: Outlier candidate detection
	double median=0;
	VectorXd Sij(grid_size*grid_size);
	int counter=0;
	for(k=-grid_size/2;k<=grid_size/2;k++){
		for(l=-grid_size/2;l<=grid_size/2;l++){
			Sij(counter)=fabs(mz2D(i,j)-mz2D(i+k,j+l));
			counter++;
		}
	}
	//MEDIAN CALCULATION
	median=find_median(Sij,counter);
	//median=Sij(counter); //TESTING
	
	//OUTLIER DETECTION
	//STAGE 1: Outlier candidate
	if(median<T1){ //not an outlier, normal data
		filtered_value=mz2D(i,j);
		return filtered_value;
	}
	
	//STAGE 2: Outlier verification
	if(median<T2){//the outlier candidate is not an outlier and a normal data
		filtered_value=mz2D(i,j);
		return filtered_value;
	}
	else{//It is an outlier, chnage the value to the nearest point
		filtered_value=mz2D(i-1,j-1);
		return filtered_value;
	}	
	
}

MatrixXd myFunction::outlier_correction_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
//void linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
	/*
	NOTE:
	deltaX=2.6 um
	deltaY=2.6 um
	image matrix size = 1624x1232
	first we have to now number of x and y since the matrix size reduces due to alicona decimation
	*/
	
	int num_of_column=0, num_of_row=0;
	double zAvg=0.0;
	int i=0, j=0;
	double yVal=0.0, xVal=0.0;
	
	//STEP 1: Calculating zAvg to close hole, num of row and col
	MatrixXd yCoord;
	calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	yCoord=calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	//cout<<yCoord;
	printf("\nNUMBER OF COLUMN= %d\n",num_of_column);
	printf("\nNUMBER OF ROW= %d\n",num_of_row);
	printf("\nAVERAGE Z= %f\n",zAvg);

	//STEP 2: Reconstructing the 2D matrix for 3D MAP.
	MatrixXd mx2D(num_of_row,num_of_column);
	MatrixXd my2D(num_of_row,num_of_column);
	MatrixXd mz2D(num_of_row,num_of_column);
	MatrixXd mz2D_temp(num_of_row,num_of_column);
	MatrixXd mz2D_filtered(num_of_row,num_of_column);
	int indexCount=0;	
	yVal=0.0;
	for(j=0;j<num_of_row;j++){ //filling the matrix is in row by row fashion (column scanning)
		yVal=yCoord(j,0);
		for(i=0;i<num_of_column;i++){
			if(fabs(yVal-m(indexCount,1))<0.000000001){
				mz2D(j,i)=m(indexCount,2); //NOTE: we only save the Z position since
															//the x and y position are in index form					
			}
			else{
				mz2D(j,i)=zAvg;
				//if there is a hole, do not increase the hole counting
			}
			mx2D(j,i)=m(indexCount,0); 
			my2D(j,i)=m(indexCount,1); 
			indexCount++;
		}
	}

	//STEP 3: Linear Gaussian Filter: ISO16610-61
	//NOTE: only processing matrix mz2D AND in this filter, Lambda is not used	
	
	//median calculation for all points to determine T1 and T2 (Threshold)
	MatrixXd median_ij(num_of_row,num_of_column);
	MatrixXd median_ij2(num_of_row,num_of_column);
	VectorXd vect_median(num_of_row*num_of_column);
	VectorXd vect_median2(num_of_row*num_of_column);

	int grid_size=5, counter_vect=0;	
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				median_ij(i,j)=0;
			}
			else{ //T1 and T2 calculation (Threshold calculation)
				median_ij(i,j)=median_calculation(mz2D,num_of_row,num_of_column,i,j,grid_size);
				median_ij2(i,j)=median_calculation2(mz2D,num_of_row,num_of_column,i,j);

				vect_median(counter_vect)=median_ij(i,j);
				vect_median2(counter_vect)=median_ij2(i,j);
				counter_vect++;
			}
		}
	}

	//T1 and T2 calculation:
	double T1=0.0;
	double T2=0.0;
	T1=find_threshold(vect_median,counter_vect);	
	T2=find_threshold(vect_median2,counter_vect);	
	
	//Outlier detection procedure
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				mz2D_filtered(i,j)=zAvg;
			}
			else{ //outlier correstion
				mz2D_filtered(i,j)=outlier_correction(mz2D,num_of_row,num_of_column,i,j,grid_size,T1,T2,zAvg);
			}
		}
	}	
	
	//STEP 4: Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;	
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}

	return m_filtered;
	
}

MatrixXd myFunction::outlier_correction_filter2(MatrixXd m, double lamda_c){ //return the filtered matrix
//void linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
	/*
	NOTE:
	deltaX=2.6 um
	deltaY=2.6 um
	image matrix size = 1624x1232
	first we have to now number of x and y since the matrix size reduces due to alicona decimation
	*/
	
	int num_of_column=0, num_of_row=0;
	double zAvg=0.0;
	int i=0, j=0;
	double yVal=0.0, xVal=0.0;
	
	//STEP 1: Calculating zAvg to close hole, num of row and col
	MatrixXd yCoord;
	calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	yCoord=calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	//cout<<yCoord;
	printf("\nNUMBER OF COLUMN= %d\n",num_of_column);
	printf("\nNUMBER OF ROW= %d\n",num_of_row);
	printf("\nAVERAGE Z= %f\n",zAvg);

	//STEP 2: Reconstructing the 2D matrix for 3D MAP.
	MatrixXd mx2D(num_of_row,num_of_column);
	MatrixXd my2D(num_of_row,num_of_column);
	MatrixXd mz2D(num_of_row,num_of_column);
	MatrixXd mz2D_temp(num_of_row,num_of_column);
	MatrixXd mz2D_filtered(num_of_row,num_of_column);
	int indexCount=0;	
	yVal=0.0;
	for(j=0;j<num_of_row;j++){ //filling the matrix is in row by row fashion (column scanning)
		yVal=yCoord(j,0);
		for(i=0;i<num_of_column;i++){
			if(fabs(yVal-m(indexCount,1))<0.000000001){
				mz2D(j,i)=m(indexCount,2); //NOTE: we only save the Z position since
															//the x and y position are in index form					
			}
			else{
				mz2D(j,i)=zAvg;
				//if there is a hole, do not increase the hole counting
			}
			mx2D(j,i)=m(indexCount,0); 
			my2D(j,i)=m(indexCount,1); 
			indexCount++;
		}
	}
	printf("\n FINISH STEP 2: SORTING DATA \n");

	//STEP 3: Linear Gaussian Filter: ISO16610-61
	//NOTE: only processing matrix mz2D AND in this filter, Lambda is not used	
	
	//median calculation for all points to determine T1 and T2 (Threshold)
	MatrixXd median_ij(num_of_row,num_of_column);
	MatrixXd median_ij2(num_of_row,num_of_column);
	VectorXd vect_median(num_of_row*num_of_column);
	VectorXd vect_median2(num_of_row*num_of_column);

	int grid_size=3, counter_vect=0;
	double avgMedian=0.0, avgMedian2=0.0;
	double sum_avgMedian,sum_avgMedian2;
	sum_avgMedian=0.0;
	sum_avgMedian2=0.0;
	double temp=0.0;
	int k=0, l=0;
	double median=0.0;
	//////Trying to put the matrix calculation here, maybe it will save time since we do nto need to send
	//////a big size of matrix to other function
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				median_ij(i,j)=0;
			}
			else{ //T1 and T2 calculation (Threshold calculation)

				VectorXd Sij(grid_size*grid_size);
				int counter=0;
				for(k=-grid_size/2 ;k<=grid_size/2;k++){
					for(l=-grid_size/2;l<=grid_size/2;l++){
						Sij(counter)=fabs(mz2D(i,j)-mz2D(i+k,j+l));
						counter++;
					}
				}	
				median=find_median(Sij,counter);
				median_ij(i,j)=median;
				//median_ij(i,j)=median_calculation(mz2D,num_of_row,num_of_column,i,j,grid_size);

				VectorXd Sij8(8); //calculating the relative distance on 8 neighbourhood
				
				Sij8(0)=fabs(mz2D(i,j)-mz2D(i-1,j-1));
				Sij8(1)=fabs(mz2D(i,j)-mz2D(i-1,j));
				Sij8(2)=fabs(mz2D(i,j)-mz2D(i-1,j+1));
				Sij8(3)=fabs(mz2D(i,j)-mz2D(i,j-1));
				Sij8(4)=fabs(mz2D(i,j)-mz2D(i,j+1));
				Sij(5)=fabs(mz2D(i,j)-mz2D(i+1,j-1));
				Sij8(6)=fabs(mz2D(i,j)-mz2D(i+1,j));
				Sij8(7)=fabs(mz2D(i,j)-mz2D(i+1,j+1));
				
				median=find_median(Sij8,8);
				median_ij2(i,j)=median;
				//median_ij2(i,j)=median_calculation2(mz2D,num_of_row,num_of_column,i,j);

				sum_avgMedian=sum_avgMedian+median_ij(i,j);
				sum_avgMedian2=sum_avgMedian2+median_ij2(i,j);

				vect_median(counter_vect)=median_ij(i,j);
				vect_median2(counter_vect)=median_ij2(i,j);
				counter_vect++;
			}
		}
	}

	avgMedian=sum_avgMedian/counter_vect;
	avgMedian2=sum_avgMedian2/counter_vect;
	avgMedian=10;
	avgMedian2=10;

	//T1 and T2 calculation:
	double T1=0.0;
	double T2=0.0;
	double thresh=0.0;
	int vector_length=counter_vect;
	//short the vector vect Ascending by Selection Sort
	for(i=0;i<vector_length-1;i++){
		temp=vect_median(i);
		for(j=i+1;j<vector_length;j++){
			if(vect_median(j)<temp){//change
				vect_median(i)=vect_median(j);
				vect_median(j)=temp;
				temp=vect_median(i);
			}
		}
	}

	int index=0;
	index=(int)0.99*vector_length;
	thresh=vect_median(index);
	T1=thresh;

	//short the vector vect Ascending by Selection Sort
	for(i=0;i<vector_length-1;i++){
		temp=vect_median2(i);
		for(j=i+1;j<vector_length;j++){
			if(vect_median2(j)<temp){//change
				vect_median2(i)=vect_median2(j);
				vect_median2(j)=temp;
				temp=vect_median2(i);
			}
		}
	}
	
	index=(int)0.99*vector_length;
	thresh=vect_median2(index);
	T2=thresh;

	//T1=avgMedian;
	//T2=avgMedian2;
	printf("\n FINISH STEP 3.1: CALCULATE MEDIAN \n");

	double filtered_value=0.0;
	//Outlier detection procedure
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				mz2D_filtered(i,j)=zAvg;
			}
			else{ //outlier correstion
				//////Trying to put the matrix calculation here, maybe it will save time since we do nto need to send
				//////a big size of matrix to other function, AND IT WORKS
				//mz2D_filtered(i,j)=outlier_correction(mz2D,num_of_row,num_of_column,i,j,grid_size,T1,T2,zAvg);
				
				int k=0, l=0;
				//STAGE 1: Outlier candidate detection
				double median=0;
				VectorXd Sij(grid_size*grid_size);
				int counter=0;
				for(k=-grid_size/2;k<=grid_size/2;k++){
					for(l=-grid_size/2;l<=grid_size/2;l++){
						Sij(counter)=fabs(mz2D(i,j)-mz2D(i+k,j+l));
						counter++;
					}
				}
				//MEDIAN CALCULATION
				median=find_median(Sij,counter);				
				
				//OUTLIER DETECTION
				//STAGE 1: Outlier candidate
				if(median<T1){ //not an outlier, normal data
					filtered_value=mz2D(i,j);
					
				}				
				//STAGE 2: Outlier verification
				else{
					if(median<T2){//the outlier candidate is not an outlier and a normal data
						filtered_value=mz2D(i,j);
						
					}
					else{//It is an outlier, chnage the value to the nearest point
						filtered_value=mz2D(i-1,j-1);
						
					}	
				}

				mz2D_filtered(i,j)=filtered_value;
				//end of median filteirng
			}
		}
	}	
	
	//STEP 4: Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;	
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}

	return m_filtered;
	
}

MatrixXd myFunction::outlier_correction_filter3(MatrixXd m, double lamda_c){ //return the filtered matrix
//void linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
	/*
	NOTE:
	deltaX=2.6 um
	deltaY=2.6 um
	image matrix size = 1624x1232
	first we have to now number of x and y since the matrix size reduces due to alicona decimation
	*/
	
	int num_of_column=0, num_of_row=0;
	double zAvg=0.0;
	int i=0, j=0;
	double yVal=0.0, xVal=0.0;
	
	//STEP 1: Calculating zAvg to close hole, num of row and col
	MatrixXd yCoord;
	calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	yCoord=calculate_row_col_zAvg(m, &num_of_row, &num_of_column, &zAvg);
	//cout<<yCoord;
	printf("\nNUMBER OF COLUMN= %d\n",num_of_column);
	printf("\nNUMBER OF ROW= %d\n",num_of_row);
	printf("\nAVERAGE Z= %f\n",zAvg);

	//STEP 2: Reconstructing the 2D matrix for 3D MAP.
	MatrixXd mx2D(num_of_row,num_of_column);
	MatrixXd my2D(num_of_row,num_of_column);
	MatrixXd mz2D(num_of_row,num_of_column);
	MatrixXd mz2D_temp(num_of_row,num_of_column);
	MatrixXd mz2D_filtered(num_of_row,num_of_column);
	int indexCount=0;	
	yVal=0.0;
	for(j=0;j<num_of_row;j++){ //filling the matrix is in row by row fashion (column scanning)
		yVal=yCoord(j,0);
		for(i=0;i<num_of_column;i++){
			if(fabs(yVal-m(indexCount,1))<0.000000001){
				mz2D(j,i)=m(indexCount,2); //NOTE: we only save the Z position since
															//the x and y position are in index form					
			}
			else{
				mz2D(j,i)=zAvg;
				//if there is a hole, do not increase the hole counting
			}
			mx2D(j,i)=m(indexCount,0); 
			my2D(j,i)=m(indexCount,1); 
			indexCount++;
		}
	}
	printf("\n FINISH STEP 2: SORTING DATA \n");

	//STEP 3: Linear Gaussian Filter: ISO16610-61
	//NOTE: only processing matrix mz2D AND in this filter, Lambda is not used	
	
	//median calculation for all points to determine T1 and T2 (Threshold)
	MatrixXd median_ij(num_of_row,num_of_column);
	
	VectorXd vect_median(num_of_row*num_of_column);
	

	int grid_size=5, counter_vect=0;
	double avgMedian=0.0, avgMedian2=0.0;
	double sum_avgMedian,sum_avgMedian2;
	sum_avgMedian=0.0;
	sum_avgMedian2=0.0;
	double temp=0.0;
	int k=0, l=0;
	double median=0.0;
	//////Trying to put the matrix calculation here, maybe it will save time since we do nto need to send
	//////a big size of matrix to other function, IT WORKS
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				median_ij(i,j)=0;
			}
			else{ //T1 and T2 calculation (Threshold calculation)

				VectorXd Sij(grid_size*grid_size);
				int counter=0;
				for(k=-grid_size/2 ;k<=grid_size/2;k++){
					for(l=-grid_size/2;l<=grid_size/2;l++){
						Sij(counter)=fabs(mz2D(i,j)-mz2D(i+k,j+l));
						counter++;
					}
				}	
				median=find_median(Sij,counter);
				median_ij(i,j)=median;
				//median_ij(i,j)=median_calculation(mz2D,num_of_row,num_of_column,i,j,grid_size);

				sum_avgMedian=sum_avgMedian+median_ij(i,j);				

				vect_median(counter_vect)=median_ij(i,j);
			
				counter_vect++;
			}
		}
	}

	avgMedian=sum_avgMedian/counter_vect;	
	avgMedian=10;

	//T1 and T2 calculation:
	double T1=0.0;
	double T2=0.0;
	double thresh=0.0;
	int vector_length=counter_vect;
	//short the vector vect_median Ascending by Selection Sort
	for(i=0;i<vector_length-1;i++){
		temp=vect_median(i);
		for(j=i+1;j<vector_length;j++){
			if(vect_median(j)<temp){//change
				vect_median(i)=vect_median(j);
				vect_median(j)=temp;
				temp=vect_median(i);
			}
		}
	}

	int index=0;
	index=(int)0.99*vector_length;
	thresh=vect_median(index);
	T1=thresh;

	printf("\n FINISH STEP 3.1: CALCULATE MEDIAN \n");

	double filtered_value=0.0;
	//Outlier detection procedure
	for(j=0;j<num_of_column;j++){
		for(i=0;i<num_of_row;i++){ //scanning row by row = y position
			if((j<=grid_size) || (j>=num_of_column-grid_size) || (i<=grid_size) || (i>=num_of_row-grid_size)){ // points on the border, ignored it by changed the value to Z avg
				mz2D_filtered(i,j)=zAvg;
				filtered_value=zAvg;
			}
			else{ //outlier correstion
				//////Trying to put the matrix calculation here, maybe it will save time since we do nto need to send
				//////a big size of matrix to other function, AND IT WORKS
				//mz2D_filtered(i,j)=outlier_correction(mz2D,num_of_row,num_of_column,i,j,grid_size,T1,T2,zAvg);
		
				//MEDIAN CALCULATION
				median=99999.9; //by defoult, the point is considered as outlier
				median=median_ij(i,j);				
				
				//OUTLIER DETECTION
				if(median<T1){ //not an outlier, normal data
					filtered_value=mz2D(i,j);					
				}								
				else{//an outlier
					mz2D_filtered(i,j)=filtered_value;//such that the outlier will be replace by the point nearby (horizontally)
				}
				
			}
		}
	}	
	
	//STEP 4: Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;	
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}

	return m_filtered;
	
}


//===================================================================================================

//====================== CYLINDRICAL FILTER =========================================================

MatrixXd myFunction::sorting_matrix(MatrixXd m, int *row, int *col){ // to close the hole, just select a neighbour point
	
	int i,j,k, flag, num_row, num_col;
	double curr_val, prev_val;

	//-----------SELECTION SORT the Matirx M-------------------------------
	double buff[3],curr[3];
	for(i=0;i<m.rows()-1;i++){
		curr[0]=m(i,0);
		curr[1]=m(i,1);
		curr[2]=m(i,2);
		for(j=i+1;j<m.rows();j++){
			if(curr[0]>=m(j,0)){//sorted according to x position 
				buff[0]=m(j,0);
				buff[1]=m(j,1);
				buff[2]=m(j,2);

				m(j,0)=curr[0];
				m(j,1)=curr[1];
				m(j,2)=curr[2];

				curr[0]=buff[0];
				curr[1]=buff[1];
				curr[2]=buff[2];
			}
		}
		m(i,0)=curr[0];
		m(i,1)=curr[1];
		m(i,2)=curr[2];
	}
	/*//writing to file the sorted matrix M
	FILE *pFile2;
	char fileName2[200];
	sprintf(fileName2,"E:\\My PhD\\PhD Thesis\\Thesis Report and Paper\\THESIS REPORT\\MEASUREMENT results\\wahyudin cylinder-SORTED TOP.txt");
	printf("\n");
	puts(fileName2);
	printf("\n");
	pFile2=fopen(fileName2,"w");
	fprintf(pFile2,"GPoint3DVector { %d n {\n",m.rows());
	for(i=0;i<m.rows();i++){	
		fprintf(pFile2,"%.9f %.9f %.9f\n",(float) m(i,0), (float) m(i,1), (float) m(i,2));
	}  
	fprintf(pFile2,"} }");
	fclose(pFile2);*/

	//--------------------------------------------------------------------
	

	//calculating number of x (col)
	flag=1;
	num_col=0;
	for(i=0;(i<m.rows()) && (flag==1);i++){
		if(i==0){ //initialization
			curr_val=m(i,0);
			prev_val=curr_val;
			num_col++;
		}
		else{
			curr_val=m(i,0);
			if(curr_val!=prev_val){
				num_col++;
				prev_val=curr_val;
			}
			
			//curr_val=m(i,1);
			//if(fabs(curr_val-m(i,1))<0.0000001){
				//num_col++;
			//}
		}
	}
	
	//calculating number of y (row)

	num_row=(int)(m.rows()/num_col);
	/*flag=1;
	num_row=0;
	for(i=0;(i<m.rows()) && (flag==1);i++){
		if(i==0){ //initialization
			curr_val=m(i,0);
			prev_val=curr_val;
			num_row++;
		}
		else{
			curr_val=m(i,0);
			if(fabs(curr_val-prev_val)<0.1){
				num_row++;
			}
			else{
				flag=0;
			}
			//curr_val=m(i,0);
			//if(fabs(curr_val-m(i,0))<0.0000001){
				//num_row++;
			//}
		}
	}*/
	
	printf("\ntotal point,col, row=%d, %d, %d\n", m.rows(),num_col, num_row);
	*row=num_row;
	*col=num_col;

	return m;
}

MatrixXd myFunction::linear_gaussian_filter_calculation(MatrixXd  mx2D,MatrixXd my2D,MatrixXd mz2D, int num_of_row, int num_of_column, double lamda_c){
	
	MatrixXd mz2D_temp(num_of_row,num_of_column);
	MatrixXd mz2D_filtered(num_of_row,num_of_column);

	//Linear Gaussian Filter: ISO16610-61
	//NOTE: only processing matrix mz2D
	const double alfa=0.44697; //from ISO 16610-61
	const double du=1, dv=80;
	//const int nWindow=lamda_c/du;	
	//lamda_c=500; //for testing
	const int nWindow=2; //for testing
	
	double sum_numerator=0.0;
	double sum_denumerator=0.0;
	double power=0.0;
	const double e=2.718281828;
	const double phi=3.1410320;

	int k=0;
	double val1=0.0, val2=0.0;

	int i,j, indexCount;

	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){ //Filtering in X-direction (column-wise)
			sum_numerator=0.0;
			sum_denumerator=0.0;
			for(k=-nWindow/2;k<nWindow/2;k++){
				power=-1*phi*((i-k)/(alfa*lamda_c))*((i-k)/(alfa*lamda_c));
				if((i-nWindow/2>=0) && (i+nWindow/2<=num_of_column-1)){ //if the range is lamda_c<n.du<lt-lamda_c (inside the 3D map matrix)
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;					
				}				
				else if((i-k < 0) || (i-k > num_of_column-1)){							
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i+k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}
				else if((i+k < 0) || (i+k > num_of_column-1)){						
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}		
				else{
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D(j,i-k)*du;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*du;
				}
				sum_numerator=sum_numerator+val1;
				sum_denumerator=sum_denumerator+val2;
			}
			//mz2D_temp(j,i)=sum_numerator/sum_denumerator;
			mz2D_filtered(j,i)=sum_numerator/sum_denumerator;
		}
	}
	/*for(i=0;i<num_of_column;i++){
		for(j=0;j<num_of_row;j++){ //Filtering in Y-direction (row-wise)
			sum_numerator=0.0;
			sum_denumerator=0.0;
			for(k=-nWindow/2;k<nWindow/2;k++){
				power=-1*phi*((j-k)/(alfa*lamda_c))*((j-k)/(alfa*lamda_c));
				if((j-nWindow/2>=0) && (j+nWindow/2<=num_of_row-1)){ //if the range is lamda_c<n.du<lt-lamda_c (inside the 3D map matrix)
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;					
				}				
				else if((j-k < 0) || (j-k > num_of_row-1)){							
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j+k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;	
				}
				else if((j+k < 0) || (j+k > num_of_row-1)){					
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;
				}						
				else{
					val1=(1/(alfa*lamda_c))*(pow(e,power))*mz2D_temp(j-k,i)*dv;
					val2=(1/(alfa*lamda_c))*(pow(e,power))*dv;	
				}
				sum_numerator=sum_numerator+val1;
				sum_denumerator=sum_denumerator+val2;				
			}
			mz2D_filtered(j,i)=sum_numerator/sum_denumerator;		
		}
	}*/
	
	// Return the filtered data
	MatrixXd m_filtered(num_of_row*num_of_column,3);
	indexCount=0;
	for(j=0;j<num_of_row;j++){
		for(i=0;i<num_of_column;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D_filtered(j,i);
			indexCount++;
		}
	}

	return m_filtered;
}

MatrixXd myFunction::robust_gaussian_regression_filter_calculation(MatrixXd mx2D,MatrixXd my2D,MatrixXd mz2D, int num_of_row, int num_of_column, double lamda_c){
	MatrixXd m_filtered;
	return m_filtered;
}

MatrixXd myFunction::cylinder_linear_gaussian_filter(MatrixXd m, double lamda_c){
	MatrixXd m_sorted;
	MatrixXd m_top_3dmap;
	MatrixXd m_filtered;

	int i,j;
	
	//sorting the matrix
	int num_row=0, num_col=0;
	m_sorted=sorting_matrix(m, &num_row, &num_col);
	
	//Converting the sorted matrix into 3D map format
	int counter;
	double curr_val=0.0, prev_val=0.0;
	VectorXd col_id(num_col);

	counter=0;
	for(i=0;i<m_sorted.rows();i++){
		if(i==0){ //initialization
			curr_val=m_sorted(i,0);
			prev_val=curr_val;
			col_id(counter)=m_sorted(i,0);
			counter++;
		}
		else{
			curr_val=m_sorted(i,0);
			if(curr_val!=prev_val){				
				prev_val=curr_val;
				col_id(counter)=m_sorted(i,0);
				counter++;
			}		
		}
	}

	MatrixXd mx2D(num_row,num_col);
	MatrixXd my2D(num_row,num_col);
	MatrixXd mz2D(num_row,num_col);
	//MatrixXd mz2D_temp(num_row,num_col);
	//MatrixXd mz2D_filtered(num_row,num_col);
	int index_count;
	int flag=0, curr_count=0;

	index_count=0;
	for(i=0;i<num_col;i++){ //column wise (filling vertically) --> converting to 3D Map
		
		flag=1;
		curr_count=index_count;
		while(flag && curr_count<num_col*num_row){
			if(col_id(i)==m_sorted(curr_count,0)){
				flag=0;
			}
			else{
				curr_count++;
			}
		}		
		index_count=curr_count;

		for(j=0;j<num_row;j++){
			if(col_id(i)==m_sorted(index_count,0)){
				mx2D(j,i)=m_sorted(index_count,0);
				my2D(j,i)=m_sorted(index_count,1);
				mz2D(j,i)=m_sorted(index_count,2);
				index_count++;
			}
			else{
				mx2D(j,i)=m_sorted(index_count-1,0);
				my2D(j,i)=m_sorted(index_count-1,1);
				mz2D(j,i)=m_sorted(index_count-1,2);
			}
		}
	}

	//--------------------------filtering the data-----------------------------------------
	m_filtered=linear_gaussian_filter_calculation(mx2D,my2D,mz2D, num_row, num_col,lamda_c);

	//writing to file the filtered matrix M
	FILE *pFile2;
	char fileName2[200];
	sprintf(fileName2,"E:\\My PhD\\PhD Thesis\\Thesis Report and Paper\\THESIS REPORT\\MEASUREMENT results\\wahyudin cylinder-FILTERED TOP.txt");
	printf("\n");
	puts(fileName2);
	printf("\n");
	pFile2=fopen(fileName2,"w");
	fprintf(pFile2,"GPoint3DVector { %d n {\n",m_filtered.rows());
	for(i=0;i<m_filtered.rows();i++){	
		fprintf(pFile2,"%.9f %.9f %.9f\n",(float) m_filtered(i,0), (float) m_filtered(i,1), (float) m_filtered(i,2));
	}  
	fprintf(pFile2,"} }");
	fclose(pFile2);
	//--------------------------------------------------------------------------------------


	//for testing to check if the conversion to 3D map format is correct
	/*m_filtered.resize(num_row*num_col,3);
	int indexCount=0; 
	for(j=0;j<num_row;j++){
		for(i=0;i<num_col;i++){
			m_filtered(indexCount,0)=mx2D(j,i);
			m_filtered(indexCount,1)=my2D(j,i);
			m_filtered(indexCount,2)=mz2D(j,i);
			indexCount++;
		}
	}*/

	printf("\n No. of points filtered: %d\n\n", m_filtered.rows());

	return m_filtered;
	//return m_sorted;
}

MatrixXd myFunction::cylinder_robust_gaussian_regression_filter(MatrixXd m, double lamda_c){
	MatrixXd m_sorted;
	MatrixXd m_top_3dmap;
	MatrixXd m_filtered;

	int i,j;
	
	//sorting the matrix
	int num_row=0, num_col=0;
	m_sorted=sorting_matrix(m, &num_row, &num_col);
	
	//Converting the sorted matrix into 3D map format
	int counter;
	double curr_val=0.0, prev_val=0.0;
	VectorXd col_id(num_col);

	counter=0;
	for(i=0;i<m_sorted.rows();i++){
		if(i==0){ //initialization
			curr_val=m_sorted(i,0);
			prev_val=curr_val;
			col_id(counter)=m_sorted(i,0);
			counter++;
		}
		else{
			curr_val=m_sorted(i,0);
			if(curr_val!=prev_val){				
				prev_val=curr_val;
				col_id(counter)=m_sorted(i,0);
				counter++;
			}		
		}
	}

	MatrixXd mx2D(num_row,num_col);
	MatrixXd my2D(num_row,num_col);
	MatrixXd mz2D(num_row,num_col);
	//MatrixXd mz2D_temp(num_row,num_col);
	//MatrixXd mz2D_filtered(num_row,num_col);
	int index_count;

	index_count=0;
	for(i=0;i<num_col;i++){ //column wise (filling vertically)
		for(j=0;j<num_row;j++){
			if(col_id(i)==m_sorted(index_count,0)){
				mx2D(j,i)=m_sorted(index_count,0);
				my2D(j,i)=m_sorted(index_count,1);
				mz2D(j,i)=m_sorted(index_count,2);
				index_count++;
			}
			else{
				mx2D(j,i)=m_sorted(index_count-1,0);
				my2D(j,i)=m_sorted(index_count-1,1);
				mz2D(j,i)=m_sorted(index_count-1,2);
			}
		}
	}

	//filtering the data
	m_filtered=robust_gaussian_regression_filter_calculation(mx2D,my2D,mz2D, num_row, num_col,lamda_c);

	return m_filtered;
}

//====================== End of Cylindrical Filter ===================================================
