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
class Simulator{
	public:
	Simulator();
	~Simulator();

	//SUPPORTING Methods
	//Random number and error generator
	float randu(long *); //The uniform random number generator
	float randn(long *); //The Gaussian (normal) random number generator
	//VectorXd mvrnd(VectorXd, MatrixXd);  //Generating variates from multi-variate Gaussian distribution, The input is variance-covariance matrix
	//MatrixXd mvrnd(MatrixXd, MatrixXd);  //Generating variates from multi-variate Gaussian distribution, The input is variance-covariance matrix
	MatrixXd mvrnd(MatrixXd);  //Generating variates from multi-variate Gaussian distribution with mean 0, The input is variance-covariance matrix
	MatrixXd errorSimulator(MatrixXd,int, double,double,double);


};


//CLASS Implementation
Simulator::Simulator(){
}

Simulator::~Simulator(){
}

//======================= RANDOM NUMBER GENERATOR =======================================
MatrixXd Simulator::errorSimulator(MatrixXd m,int mode, double s,double n,double r){
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
MatrixXd Simulator::mvrnd(MatrixXd sigma){ //mean is zero
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

	//initializing m_randn matrix SEEDING
	//long initial=-1000000; //should be a negative integer
	long initial=(long) time(NULL);
	if(initial>0){
		initial=initial*-1;
	}
	else if(initial==0){
		initial=-1000000;
	}
	
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

float   Simulator::randn(long   *idum){ 
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

float   Simulator::randu(long   *idum){ 
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

