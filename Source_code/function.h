//#include <Eigen/SVD>
void setRotoTranslationMatrix(MatrixXd &T, double dx, double dy, double dz,double c11, double c12, double c13, double c21, double c22, double c23, double c31, double c32, double c33){
	T(0,0)=c11; T(0,1)=c12; T(0,2)=c13; T(0,3)=dx;
	T(1,0)=c21; T(1,1)=c22; T(1,2)=c23; T(1,3)=dy;
	T(2,0)=c31; T(2,1)=c32; T(2,2)=c33; T(2,3)=dz;
	T(3,0)=0;   T(3,1)=0;   T(3,2)=0;   T(3,3)=1;
}
//====================== SETTING TRANSLATION MATRIX ===========================
void setTranslationMatrix(MatrixXd &T, double dx, double dy, double dz){
	T(0,0)=1; T(0,1)=0; T(0,2)=0; T(0,3)=dx;
	T(1,0)=0; T(1,1)=1; T(1,2)=0; T(1,3)=dy;
	T(2,0)=0; T(2,1)=0; T(2,2)=1; T(2,3)=dz;
	T(3,0)=0; T(3,1)=0; T(3,2)=0; T(3,3)=1;
}

//======================= CHAOS ALGORITHM ===================================
//Using Chaotic Optimization Algorithm
//function to improve the initial point by chaos optimization algorithm
double r_norm_calc_sphere(MatrixXd m, double p01,double p02,double p03,double p04){
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

double random_number(){//Sjould be improved and changed
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

void chaos_initial_point_sphere(MatrixXd m, double *p01, double *p02, double *p03, double *p04){
	int counterPoint=0, k=0 , r=0, kMax=0, rMax=0, paramDim=4;
	int i=0;
	double limit=0.0, ub=0.0, lb=0.0, lamda=0, gamma=0.0, jstar=0.0, jk=0.0, rnorm=0.0;
	double t0=0.0, tk[4], tstar[4], a[4], b[4], ar[4], br[4], pstar[4], pk[4];
	double arOld[4], arNew[4], brOld[4], brNew[4], p0[4];

	//double fixPx=0.0, fixPy=0.0;
	//fixPx=*p01;
	//fixPy=*p02;

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

	
	//------------------ STEP 1: INITIALIZATION ---------------------------
	
	for(i=0;i<paramDim;i++){
		t0=random_number();
		while(t0==0.0|| t0==0.25 || t0==0.5 || t0==0.75 || t0==1.0){ //possible infinite loop
			t0=random_number();
			
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
	//printf("jstar=%lf\n",jstar);

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
				t0=random_number();
				while(t0==0 || t0==0.25 || t0==0.5 || t0==0.75 || t0==1){
					t0=random_number();
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
void plane_fitting(MatrixXd m, double *px, double *py, double *pz, double *n1, double *n2, double *n3){
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

void line_fitting(MatrixXd m, double *px, double *py, double *pz, double *n1, double *n2, double *n3){
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
	
	/*printf("\n\nThe matrix: \n");
	cout<<m_translated;
	printf("\n\nU: \n");
	cout<<U;
	printf("\n\nS: \n");
	cout<<S;
	printf("\n\nV: \n");
	cout<<V;*/
	
}


void sphere_fitting(MatrixXd m, double *px, double *py, double *pz, double *r){//Lavenberg-Marquardt (LM) Algorithm
	double xi=0.0,yi=0.0,zi=0.0, avgX=0.0,avgY=0.0,avgZ=0.0,sumX=0.0,sumY=0.0,sumZ=0.0;
	int counter=0, counterPoint=0, i=0;
	double maxX=0.0, maxY=0.0, maxZ=0.0;
	double minX=0.0, minY=0.0, minZ=0.0;
	double p0[4],p[4], p_opt[4];

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
	p0[2]=avgZ;

	//--------------------------------
	p0[0]=avgX;
	p0[1]=avgY;
	//p0[2]=maxZ-0.0025;
	//p0[3]=0.0025;
	p0[2]=maxZ-2500;
	p0[3]=2500;
	//---------------------------------

	printf("\nP0 = %lf %lf %lf %lf \n", p0[0], p0[1], p0[2], p0[3]);
	//chaos_initial_point_sphere(m,&p0[0],&p0[1],&p0[2],&p0[3]);	
	printf("\nP0 = %lf %lf %lf %lf \n", p0[0], p0[1], p0[2], p0[3]);
	
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
			if(counter2>100){
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
		if(counter1>100){
			stop1=1;
		}
	}

	*px=p_opt[0];
	*py=p_opt[1];
	*pz=p_opt[2];
	*r=p_opt[3];
	
}

void sphere_fitting_25_points(MatrixXd m_all, MatrixXd m, double *px, double *py, double *pz, double *r){//Lavenberg-Marquardt (LM) Algorithm
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
	p0[2]=avgZ;
	
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
	//---------------------------------
	
	counterPoint=0;
	counterPoint=m.rows();

	printf("\nP0 = %lf %lf %lf %lf \n", p0[0], p0[1], p0[2], p0[3]);
	//chaos_initial_point_sphere(m,&p0[0],&p0[1],&p0[2],&p0[3]);	
	printf("\nP0 = %lf %lf %lf %lf \n", p0[0], p0[1], p0[2], p0[3]);
	
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

void cylinder_fitting(MatrixXd m, double *px, double *py, double *pz, double *a1,double *a2,double *a3, double *r){//Lavenberg-Marquardt (LM) Algorithm
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

	p0[3]=(((maxX-minX)/2)+((maxY-minY)/2))/2;	
	p0[0]=avgX;
	p0[1]=avgY;
	p0[2]=avgZ;

	//--------------------------------*/

	//Initial point
	p0[0]=*px;
	p0[1]=*py;
	p0[2]=*pz;
	p0[3]=*a1;
	p0[4]=*a2;
	p0[5]=*a3;
	p0[6]=*r;
		
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
		if(counter1>100){
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
	
}


//========================== FILTERING ALGORITMH =========================================
/*
NOTE: The filtering is based on ISO 16610 series, especially for areal (2D) fultering.
*/
//void calculate_row_col_zAvg(MatrixXd m, int *nrow, int *ncol, double *zAvg1){
MatrixXd calculate_row_col_zAvg(MatrixXd m, int *nrow, int *ncol, double *zAvg1){
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

MatrixXd linear_gaussian_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
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

MatrixXd robust_gaussian_regression_filter(MatrixXd m, double lamda_c){ //return the filtered matrix

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

MatrixXd test_return_matrix(MatrixXd m,int X, int y){
	MatrixXd x(3,3);

	x(0,0)=1; x(0,1)=2; x(0,2)=3;
	x(1,0)=4; x(1,1)=5; x(1,2)=6;
	x(2,0)=7; x(2,1)=8; x(2,2)=9;

	return x;

}

//====================== OUTLIER CORRECTION FILTER ==================================================
double find_threshold(VectorXd vect,int vector_length){
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

double find_median(VectorXd &Sij, int vector_length){// function to find a median from a vector of data set
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

double median_calculation(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j,int grid_size){
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

double median_calculation2(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j){
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

double outlier_correction(MatrixXd mz2D,int no_of_row, int no_of_column,int i,int j,int grid_size, double T1, double T2, double zAvg){
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



MatrixXd outlier_correction_filter(MatrixXd m, double lamda_c){ //return the filtered matrix
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

MatrixXd outlier_correction_filter2(MatrixXd m, double lamda_c){ //return the filtered matrix
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

MatrixXd outlier_correction_filter3(MatrixXd m, double lamda_c){ //return the filtered matrix
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

MatrixXd sorting_matrix(MatrixXd m, int *row, int *col){ // to close the hole, just select a neighbour point
	
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

MatrixXd linear_gaussian_filter_calculation(MatrixXd  mx2D,MatrixXd my2D,MatrixXd mz2D, int num_of_row, int num_of_column, double lamda_c){
	
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

MatrixXd robust_gaussian_regression_filter_calculation(MatrixXd mx2D,MatrixXd my2D,MatrixXd mz2D, int num_of_row, int num_of_column, double lamda_c){
	MatrixXd m_filtered;
	return m_filtered;
}

MatrixXd cylinder_linear_gaussian_filter(MatrixXd m, double lamda_c){
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

MatrixXd cylinder_robust_gaussian_regression_filter(MatrixXd m, double lamda_c){
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


