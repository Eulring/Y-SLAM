#include "Yslam/g2opose.h"


namespace Yslam
{


G2OPose::G2OPose() {}

void G2OPose::updateFrames(vector<Frame>& frames)
{
	ifstream fin("result.g2o");

	while(!fin.eof())
	{
		string name;
		fin>>name;
		if(name == "VERTEX_SE3:QUAT")
		{
			int idx;
			double t[7];
			fin>>idx;
			for (int j=0;j<7;j++) fin>>t[j];
			Eigen::Quaterniond q;
			Vector3d tr;

			tr(0,0)=t[0];tr(1,0)=t[1];tr(2,0)=t[2];
			q.x()=t[3];q.y()=t[4];q.z()=t[5];q.w()=t[6];

			SE3 SE3_qt(q,tr);
			//cout<<(frames[idx].T_f_w_.log()-SE3_qt.log()).transpose()<<endl;
			Eigen::Matrix3d R;
			//Eigen::Vector3d tr;
			R=SE3_qt.rotation_matrix();
			//t=SE3_qt.translation();

			eigen2cv(R,frames[idx].R_);
			eigen2cv(tr,frames[idx].t_);

		}

	}
}

void G2OPose::poseGraphOpt2(vector<Frame>& frames)
{
	ifstream fin("init.g2o");

	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    int vertexCnt = 0, edgeCnt = 0;


	while ( !fin.eof() )
    {
        string name;
        fin>>name;
        if ( name == "VERTEX_SE3:QUAT" )
        {

            g2o::VertexSE3* v = new g2o::VertexSE3();
            int index = 0;
            fin>>index;
            v->setId( index );
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if ( index==0 )
                v->setFixed(true);
			//if(vertexCnt==2) cout<<(*(v))<<endl;
        }
        else if ( name=="EDGE_SE3:QUAT" )
        {

            g2o::EdgeSE3* e = new g2o::EdgeSE3();
            int idx1, idx2;
            fin>>idx1>>idx2;
            e->setId( edgeCnt++ );
            e->setVertex( 0, optimizer.vertices()[idx1] );
            e->setVertex( 1, optimizer.vertices()[idx2] );
            e->read(fin);
            optimizer.addEdge(e);
        }
        if ( !fin.good() ) break;
    }
	//int  x; cin>>x;
    cout<<"read total "<<vertexCnt<<" vertices, "<<edgeCnt<<" edges."<<endl;

    cout<<"prepare optimizing ..."<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    cout<<"calling optimizing ..."<<endl;
    optimizer.optimize(30);

    cout<<"saving optimization results ..."<<endl;
    optimizer.save("result.g2o");


	fin.close();
}


void G2OPose::writeG2oFile(vector<Frame>& frames)
{
	ofstream fout("init.g2o");
	for(int i=0;i<frames.size()-1;i++)
	{
		fout<<"VERTEX_SE3:QUAT ";
		fout<<i<<" ";
		//SE3 tmp=frames[i].T_f_w_;

		Eigen::Matrix3d R;
		Eigen::Vector3d t;
		cv2eigen(frames[i].R_,R);
		cv2eigen(frames[i].t_,t);
		Eigen::Quaterniond q(R);

		fout<<t(0,0)<<" "<<t(1,0)<<" "<<t(2,0)<<" ";
		fout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w();
		fout<<"\n";
	}

	for(int i=1;i<frames.size();i++)
	{
		fout<<"EDGE_SE3:QUAT ";

		if(i==frames.size()-1)
			fout<<i-1<<" "<<0<<" ";
		else
			fout<<i-1<<" "<<i<<" ";

		Eigen::Matrix3d R_delta;
		Eigen::Vector3d t_delta;
		cv2eigen(frames[i].R_delta_,R_delta);
		cv2eigen(frames[i].t_delta_,t_delta);


		Eigen::Quaterniond q(R_delta);
		Vector3d t(t_delta);
		fout<<t(0,0)<<" "<<t(1,0)<<" "<<t(2,0)<<" ";
		fout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w();

		for(int j=1;j<=6;j++)
			for(int k=j;k<=6;k++)
			{
				fout<<" ";
				if(j==k)
				{
					if(k>3) fout<<40000;
					else fout<<10000;
				}
				else fout<<0;

			}
		fout<<"\n";
	}
	fout.close();
}
//EDGE_SE3:QUAT 292 343 -0.328865 5.24825 -6.09388 0.0608007 0.0269939
//VERTEX_SE3:QUAT 0 -0.125664
void G2OPose::poseGraphOpt(vector<Frame>& frames)
{
	/*
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> Block;  // 6x6 BlockSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( linearSolver );      // 矩阵块求解器
    // 梯度下降方法，从GN, LM, DogLeg 中选
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm( solver );   // 设置求解器

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量

	for (int i=0;i<frames.size()-1;i++)
	{
		g2o::VertexSE3* v = new g2o::VertexSE3();
		v->setId(i);

		v->setEstimate(
			g2o::SE3Quat( frames[i].T_f_w_.rotation_matrix(),
				frames[i].T_f_w_.translation() )
		);
		optimizer.addVertex(v);
		vertexCnt++;
		if(i==0)
			v->setFixed(true);
	}

	for(int i=1;i<frames.size();i++)
	{
		g2o::EdgeSE3* e = new g2o::EdgeSE3();
		int idx1=i-1,idx2=i;
		if(i==frames.size()-1) idx2=0;
		e->setId(edgeCnt++);
		e->setVertex( 0, optimizer.vertices()[idx1] );
		e->setVertex( 1, optimizer.vertices()[idx2] );
		e->setMeasurement(
			g2o::SE3Quat( frames[i].T_f_f_.rotation_matrix(),
				frames[i].T_f_f_.translation() )
		);
		Eigen::Matrix<double,6,6> inf_mat;
		for(int i=0;i<6;i++) for(int j=0;j<6;j++)
		{
			if(i==j) {
				inf_mat(i,i)=10000;
				if(i>=3) inf_mat(i,i)=40000;
			}
			else inf_mat(i,j)=0;
		}
		e->setInformation(inf_mat);
		optimizer.addEdge(e);
	}

	optimizer.setVerbose(true);
    optimizer.initializeOptimization();
	optimizer.optimize(10);

	optimizer.save("result.g2o");

	ifstream fin("result.g2o");

	while(!fin.eof())
	{
		string name;
		fin>>name;
		if(name == "VERTEX_SE3:QUAT")
		{
			int idx;
			double t[7];
			fin>>idx;
			for (int j=0;j<7;j++) fin>>t[j];
			Eigen::Quaterniond q;
			Vector3d tr;

			tr(0,0)=t[0];tr(1,0)=t[1];tr(2,0)=t[2];
			q.x()=t[3];q.y()=t[4];q.z()=t[5];q.w()=t[6];

			SE3 SE3_qt(q,tr);
			//cout<<(frames[idx].T_f_w_.log()-SE3_qt.log()).transpose()<<endl;
			frames[idx].T_f_w_=SE3_qt;
			//SE3_Rt.log()
		}

	}

	/*
	ostrstream os;
	optimizer.save(os);
	string str;
	while(str=getline(os)) {
		str=o
	}
	cout<<os.str()<<endl;
	*/
}


}
