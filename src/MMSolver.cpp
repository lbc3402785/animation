#pragma once
#define TODO
#define errorcout cout
#define debugcout cout
#define CV_AA -1
#include "MMSolver.h"
#include "ceresnonlinear.hpp"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"
void MMSolver::Initialize(string file, string file2)
{
    FM.Initialize(file, false);
    //FMFull.Initialize(file2, FM.Mean, true);
    FMFull.Initialize(file2, true);


    if (FIXFIRSTSHAPE)
    {
        FM.SB.col(0) *= 0;
        FMFull.SB.col(0) *= 0;
    }
}
MatF MMSolver::PerspectiveProjection(ProjectionParameters& p, MatF& model_points)
{
    MatF rotated=model_points*p.R.transpose();
    rotated.col(0).array() += p.tx;
    rotated.col(1).array() += p.ty;
    rotated.col(2).array() += p.tz;
    MatF zs=rotated.col(2);
//    MatF fs=MatF::Ones(zs.rows(),zs.cols())*fx;
//    MatF rs=fs.array()/zs.array();
//    MatF xs=rotated.col(0);
//    MatF ys=rotated.col(1);
//    xs=rs.array()*xs.array();
//    ys=rs.array()*ys.array();
//    MatF pro=MatF::Zero(rotated.rows(),2);
//    pro.col(0)=xs;
//    pro.col(1)=ys;
    MatF pro=rotated.leftCols(2)*fx;
    pro.col(0).array()/=zs.array();
    pro.col(1).array()/=zs.array();
    return std::move(pro);
}
MatF MMSolver::Transform(ProjectionParameters& p, MatF model_points)
{
    MatF rotated=model_points*p.R.transpose();
    rotated.col(0).array() += p.tx;
    rotated.col(1).array() += p.ty;
    rotated.col(2).array() += p.tz;
    return rotated;
}
MatF MMSolver::SolveShapePerspective(ProjectionParameters p, MatF image_points, MatF M, MatF SB, float lambda)
{
    //cout << Shape(SB) << endl;

    MatF rotated = Transform(p, M);//Nx3
    MatF zs=rotated.col(2);
    MatF xs=rotated.col(0);
    xs*=fx;
    MatF ys=rotated.col(1);
    ys*=fx;
    MatF imageXs=image_points.col(0);
    imageXs=imageXs.array()*zs.array();
    MatF imageYs=image_points.col(1);
    imageYs=imageYs.array()*zs.array();
    MatF errXs=imageXs-xs;
    MatF errYs=imageYs-ys;
    MatF error = MatF::Zero(image_points.rows(),image_points.cols());
    error.col(0)=errXs;
    error.col(1)=errYs;
    error = Reshape(error, 1);

    int N = M.rows();
    int N2 = SB.rows();
    int L = SB.cols();

    assert(N2 == N * 3);
    auto sTx = p.tx * p.s;
    auto sTy = p.ty * p.s;
    MatF SBX(N * 2, L);
    for (size_t i = 0; i < N; i++)
    {
        MatF SBRotation= p.R * SB.block(i * 3, 0, 3, L);//3xL
        MatF xs=SBRotation.row(0);//1xL
        MatF ys=SBRotation.row(1);//1xL
        float zc=rotated(i,2);
        xs*=fx;
        ys*=fx;
        SBX.block(i * 2, 0, 1, L)=xs;
        SBX.block(i * 2+1, 0, 1, L)=ys;
        /*SBX.row(i * 2) .array()= sTx;
        SBX.row(i * 2+1).array() = sTy;*/
    }

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(2 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(2 * SkipList[i] + 0, 0) = WEIGHT;
            W(2 * SkipList[i] + 1, 0) = WEIGHT;
        }
        SBX = W.asDiagonal() * SBX;
        error = W.asDiagonal() * error;
    }

    auto X = SolveLinear(SBX/fx, error/fx, lambda);

    return X;
    //MatF rotated = (model_points + Ax) * R;
}

MatF MMSolver::SolveShapePerspective1(ProjectionParameters p, MatF image_points, MatF M, MatF SB, VectorXf rv)
{
    MatF rotated = Transform(p, M);//Nx3
    MatF zs=rotated.col(2);
    MatF xs=rotated.col(0);
    xs*=fx;
    MatF ys=rotated.col(1);
    ys*=fx;
    MatF imageXs=image_points.col(0);
    imageXs=imageXs.array()*zs.array();
    MatF imageYs=image_points.col(1);
    imageYs=imageYs.array()*zs.array();
    MatF errXs=imageXs-xs;
    MatF errYs=imageYs-ys;
    MatF error = MatF::Zero(image_points.rows(),image_points.cols());
    error.col(0)=errXs;
    error.col(1)=errYs;
    error = Reshape(error, 1);

    int N = M.rows();
    int N2 = SB.rows();
    int L = SB.cols();

    assert(N2 == N * 3);
    auto sTx = p.tx * p.s;
    auto sTy = p.ty * p.s;
    MatF SBX(N * 2, L);
    for (size_t i = 0; i < N; i++)
    {
        MatF SBRotation= p.R * SB.block(i * 3, 0, 3, L);//3xL
        MatF xs=SBRotation.row(0);//1xL
        MatF ys=SBRotation.row(1);//1xL
        float zc=rotated(i,2);
        xs*=fx;
        ys*=fx;
        SBX.block(i * 2, 0, 1, L)=xs;
        SBX.block(i * 2+1, 0, 1, L)=ys;
        /*SBX.row(i * 2) .array()= sTx;
        SBX.row(i * 2+1).array() = sTy;*/
    }

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(2 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(2 * SkipList[i] + 0, 0) = WEIGHT;
            W(2 * SkipList[i] + 1, 0) = WEIGHT;
        }
        SBX = W.asDiagonal() * SBX;
        error = W.asDiagonal() * error;
    }

    auto X = SolveLinear(SBX/fx, error/fx, rv);

    return X;
}
MatF MMSolver::SolveShape(ProjectionParameters p, MatF image_points, MatF M, MatF SB, float lambda)
{
    //cout << Shape(SB) << endl;

    MatF R = p.R.transpose()  * p.s;
    R = R.block(0, 0, 3, 2);

    MatF rotated = Projection(p, M);

    MatF error = image_points - rotated;
    error = Reshape(error, 1);

    int N = M.rows();
    int N2 = SB.rows();
    int L = SB.cols();

    assert(N2 == N * 3);
    auto sTx = p.tx * p.s;
    auto sTy = p.ty * p.s;
    MatF SBX(N * 2, L);
    MatF Rt = R.transpose();
    for (size_t i = 0; i < N; i++)
    {
        SBX.block(i * 2, 0, 2, L) =  p.R * SB.block(i * 3, 0, 3, L);
        /*SBX.row(i * 2) .array()= sTx;
        SBX.row(i * 2+1).array() = sTy;*/
    }

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(2 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(2 * SkipList[i] + 0, 0) = WEIGHT;
            W(2 * SkipList[i] + 1, 0) = WEIGHT;
        }
        SBX = W.asDiagonal() * SBX;
        error = W.asDiagonal() * error;
    }

    auto X = SolveLinear(SBX, error, lambda);

    //cout << (error - SBX * X).norm() << endl;

    return X;
    //MatF rotated = (model_points + Ax) * R;
}
MatF MMSolver::SolveShape2(ProjectionParameters p, MatF pre, MatF cur, MatF SB, float lambda)
{
    //cout << Shape(SB) << endl;



    MatF error = cur - pre;
    //cout << error << endl;
    error = Reshape(error, 1);

    int N = cur.rows();
    int N2 = SB.rows();
    int L = SB.cols();

    assert(N2 == N * 3);
    MatF SBX(N * 3, L);
    for (size_t i = 0; i < N; i++)
    {
        SBX.block(i * 3, 0, 3, L) = p.R * SB.block(i * 3, 0, 3, L);
    }

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(3 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(3 * SkipList[i] + 0, 0) = WEIGHT;
            W(3 * SkipList[i] + 1, 0) = WEIGHT;
            W(3 * SkipList[i] + 2, 0) = WEIGHT;
        }
        SBX = W.asDiagonal() * SBX;
        error = W.asDiagonal() * error;
    }

    auto X = SolveLinear(SBX, error, lambda);

    //cout << (error - SBX * X).norm() << endl;

    return X;
    //MatF rotated = (model_points + Ax) * R;
}
MatF MMSolver::SolveShape3(ProjectionParameters p, MatF pre, MatF cur, MatF SB, VectorXf rv)
{
    //cout << Shape(SB) << endl;



    MatF error = cur - pre;
    //cout << error << endl;
    error = Reshape(error, 1);

    int N = cur.rows();
    int N2 = SB.rows();
    int L = SB.cols();

    assert(N2 == N * 3);
    MatF SBX(N * 3, L);
    for (size_t i = 0; i < N; i++)
    {
        SBX.block(i * 3, 0, 3, L) = p.R * SB.block(i * 3, 0, 3, L);
    }

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(3 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(3 * SkipList[i] + 0, 0) = WEIGHT;
            W(3 * SkipList[i] + 1, 0) = WEIGHT;
            W(3 * SkipList[i] + 2, 0) = WEIGHT;
        }
        SBX = W.asDiagonal() * SBX;
        error = W.asDiagonal() * error;
    }

    auto X = SolveLinear(SBX, error, rv);

    //cout << (error - SBX * X).norm() << endl;

    return X;
    //MatF rotated = (model_points + Ax) * R;
}

ProjectionParameters MMSolver::SolveProjectionNonlinear(MatF image_points, MatF model_points){
    using namespace fitting;
    int N = image_points.rows();
    Eigen::Quaternionf q(params.R);
    std::vector<double> cameraRotation(4,0.0);
    cameraRotation[0] = q.w();
    cameraRotation[1] = q.x();
    cameraRotation[2] = q.y();
    cameraRotation[3] = q.z();
    std::vector<double> translation(3,100.0);
    translation[0]=params.tx;
    translation[1]=params.ty;
    translation[2]=params.tz;
    ceres::Problem problem;
    for(int i=0;i<N;i++){
        Eigen::Vector3f srcKeyPoint= model_points.row(i);
        Eigen::Vector3f dstKeyPoint(image_points(i,0),image_points(i,1),1);
        fitting::Key2DPointCost* cost=new fitting::Key2DPointCost(fx,srcKeyPoint,dstKeyPoint);
        ceres::CostFunction* costFunction=new ceres::AutoDiffCostFunction<fitting::Key2DPointCost,2,4,3>(cost);
        problem.AddResidualBlock(costFunction,/*new ceres::CauchyLoss(0.5)*/NULL,&cameraRotation[0],&translation[0]);
    }
    ceres::QuaternionParameterization* cameraFitQuaternionParameterisation = new ceres::QuaternionParameterization();
    problem.SetParameterization(&cameraRotation[0], cameraFitQuaternionParameterisation);
    problem.SetParameterUpperBound(&translation[0],2,2000);
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::SPARSE_SCHUR;
    solverOptions.num_threads = 1;
    //solverOptions.max_num_iterations=500;
    solverOptions.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solverSummary;
    ceres::Solve(solverOptions, &problem, &solverSummary);
    //std::cout << solverSummary.BriefReport() << "\n";
    Eigen::Quaternion<double> qd(cameraRotation[0],cameraRotation[1],cameraRotation[2],cameraRotation[3]);
    Eigen::Matrix<double, 3, 1> t(translation[0],translation[1],translation[2]);
    Eigen::Matrix<double,3,3> Rotation=qd.toRotationMatrix();
    Eigen::Matrix<float,3,3> Rf=Rotation.cast<float>();
    Eigen::Matrix<float,3,1> tf=t.cast<float>();

    return {Rf,tf(0,0),tf(1,0),tf(2,0),1.0f};
}
ProjectionParameters MMSolver::SolveProjection(MatF image_points, MatF model_points)
{
    //########## Mean should be subtracted from model_points ############

    Matrix<float, 1, Eigen::Dynamic> Mean = GetMean(model_points);
    model_points.rowwise() -= Mean;


    using Eigen::Matrix;
    int N = image_points.rows();

    assert(image_points.rows() == model_points.rows());
    assert(2 == image_points.cols());
    assert(3 == model_points.cols());

    model_points.conservativeResize(N, 4);
    model_points.col(3).setOnes();

    Matrix<float, Eigen::Dynamic, 8> A = Matrix<float, Eigen::Dynamic, 8>::Zero(2 * N, 8);
    for (int i = 0; i < N; ++i)
    {
        Eigen::Vector4f P = model_points.row(i);// .transpose();//Eigen::Vector4f();
        A.block<1, 4>(2 * i, 0) = P;       // even row - copy to left side (first row is row 0)
        A.block<1, 4>((2 * i) + 1, 4) = P; // odd row - copy to right side
    } // 4th coord (homogeneous) is already 1

    //Matrix<float, 1, Eigen::Dynamic> MeanX = image_points.colwise().mean();
    //image_points.rowwise() -= MeanX;

    MatF b = Reshape(image_points, 1);

    if (USEWEIGHT)
    {
        Matrix<float, Eigen::Dynamic, 1> W = Matrix<float, Eigen::Dynamic, 1>::Ones(2 * N, 1);
        for (size_t i = 0; i < SkipList.size(); i++)
        {
            W(2 * SkipList[i] + 0, 0) = WEIGHT;
            W(2 * SkipList[i] + 1, 0) = WEIGHT;
        }
        A = W.asDiagonal() * A;
        b = W.asDiagonal() * b;
    }

    const Matrix<float, 8, 1> k = SolveLinear(A, b); // resulting affine matrix (8x1)
    // Extract all values from the estimated affine parameters k:
    const Eigen::Vector3f R1 = k.segment<3>(0);
    const Eigen::Vector3f R2 = k.segment<3>(4);
    Eigen::Matrix3f R;
    Eigen::Vector3f r1 = R1.normalized(); // Not sure why R1.normalize() (in-place) produces a compiler error.
    Eigen::Vector3f r2 = R2.normalized();
    R.block<1, 3>(0, 0) = r1;
    R.block<1, 3>(1, 0) = r2;
    R.block<1, 3>(2, 0) = r1.cross(r2);
    float sTx = k(3);
    float sTy = k(7);

    //sTx += Mean(0);
    //sTy += Mean(1);

    const auto s = (R1.norm() + R2.norm()) / 2.0f;


    Eigen::Matrix3f R_ortho = Orthogonalize(R);

    //    std::cout << "R:" << R << std::endl;
    //    std::cout << "R_ortho:" << R_ortho << std::endl;
    MatF T = Mean * R_ortho.transpose();
    // Remove the scale from the translations:
    auto t1 = sTx / s - T(0);
    auto t2 = sTy / s - T(1);

    auto error = (A*k - b).norm();

    //    std::cout << "TTS:" << t1 << " " << t2 << " " << s << std::endl;
    //    std::cout << "error:" << error << std::endl;

    return ProjectionParameters{ R_ortho, t1, t2,0, s };
}
void MMSolver::Solve(MatF& KP)
{
    MatF Face = FM.Face;
    MatF S = Face * 0;
    MatF E = Face * 0;


    float Lambdas[7] = { 100.0, 30.0, 10.0, 5.0,4.0,3.0,2.0};


    for (size_t i = 0; i < 4; i++)
    {
        params = SolveProjection(KP, Face);

        if (FixShape)
        {
            SX = SX0;
        }
        else
        {
            SX = SolveShape(params, KP, FM.Face + E, FM.SB, Lambdas[i]*5);
            if (FIXFIRSTSHAPE)
            {
                SX(0, 0) = 0;
            }
        }
        MatF FaceS = FM.SB * SX;
        S = Reshape(FaceS, 3);

        EX = SolveShape(params, KP, FM.Face + S, FM.EB, Lambdas[i]*1);

        MatF FaceE = FM.EB * EX;
        E = Reshape(FaceE, 3);

        Face = FM.Face + S + E;


    }

}
void MMSolver::SolvePerspective(MatF& KP)
{
    MatF Face = FM.Face;
    MatF S = Face * 0;
    MatF E = Face * 0;
    float Lambdas[7] = { 100.0, 30.0, 10.0, 5.0,4.0,3.0,2.0};
    for (size_t i = 0; i < 4; i++)
    {
        params = SolveProjectionNonlinear(KP, Face);
        if (FixShape)
        {
            SX = SX0;
        }
        else
        {
            SX = SolveShapePerspective(params, KP, FM.Face + E, FM.SB, Lambdas[i]*5);
            if (FIXFIRSTSHAPE)
            {
                SX(0, 0) = 0;
            }
        }
        MatF FaceS = FM.SB * SX;
        S = Reshape(FaceS, 3);
        EX = SolveShapePerspective(params, KP, FM.Face + S, FM.EB, Lambdas[i]*1);
        MatF FaceE = FM.EB * EX;
        E = Reshape(FaceE, 3);
        Face = FM.Face + S + E;

    }
    //    std::cout << "SX:" << SX << std::endl;
    //    std::cout << "EX:" << EX << std::endl;

}
void MMSolver::SolvePerspective1(MatF& KP,VectorXf rsv,VectorXf rev)
{
    MatF Face = FM.Face;
    MatF S = Face * 0;
    MatF E = Face * 0;
    //    float Lambdas[7] = { 100.0, 30.0, 10.0, 5.0,4.0,3.0,2.0};
    float lambdaSX[4]={100,20.0,2.0,1.0f};
    float lambdaEX[4]={1.0,0.2,0.04,0.02f};
    params.tx=0;
    params.ty=0;
    params.tz=450;
    for (size_t i = 0; i < 4; i++)
    {
        params = SolveProjectionNonlinear(KP, Face);
        if (FixShape)
        {
            SX = SX0;
        }
        else
        {
            //            SX = SolveShapePerspective(params, KP, FM.Face + E, FM.SB, Lambdas[i]*5);
            SX = SolveShapePerspective1(params, KP, FM.Face + E, FM.SB, lambdaSX[i]*rsv);
            if (FIXFIRSTSHAPE)
            {
                SX(0, 0) = 0;
            }
        }
        MatF FaceS = FM.SB * SX;
        S = Reshape(FaceS, 3);
        EX = SolveShapePerspective1(params, KP, FM.Face + S, FM.EB,lambdaEX[i]*rev);
        MatF FaceE = FM.EB * EX;
        E = Reshape(FaceE, 3);
        Face = FM.Face + S + E;

    }
    //    std::cout << "SX:" << SX << std::endl;
    //    std::cout << "EX:" << EX << std::endl;

}
void MMSolver::SolvePerspective2(MatF& KP)
{
    using namespace fitting;
    int N = KP.rows();
    Eigen::Quaternionf q(params.R);
    std::vector<double> cameraRotation(4,0.0);
    cameraRotation[0] = q.w();
    cameraRotation[1] = q.x();
    cameraRotation[2] = q.y();
    cameraRotation[3] = q.z();
    std::vector<double> translation(3,0.0);
    translation[0]=params.tx;
    translation[0]=params.ty;
    translation[2]=450;
    constexpr int  num_coeffs_fitting=199;
    constexpr int  num_blendshapes=100;
    std::vector<double> shape_coefficients(num_coeffs_fitting,0);
    std::vector<double> blendshape_coefficients(num_blendshapes,0);
    ceres::Problem problem;
    for(int i=0;i<N;i++){
        Eigen::Vector3f dstKeyPoint(KP(i,0),KP(i,1),1);
        fitting::LandmarkCost* cost=new fitting::LandmarkCost(FM,fx,i,dstKeyPoint);
        ceres::CostFunction* costFunction=new ceres::AutoDiffCostFunction<fitting::LandmarkCost,2,4,3,num_coeffs_fitting,num_blendshapes>(cost);
        problem.AddResidualBlock(costFunction,/*new ceres::CauchyLoss(0.5)*/NULL,&cameraRotation[0],&translation[0],&shape_coefficients[0],&blendshape_coefficients[0]);

    }
    problem.SetParameterLowerBound(&translation[0],2,100);
    problem.SetParameterUpperBound(&translation[0],2,2000);

    //    problem.SetParameterBlockConstant(&shape_coefficients[0]);
    //    problem.SetParameterBlockConstant(&blendshape_coefficients[0]);
    ceres::QuaternionParameterization* cameraFitQuaternionParameterisation = new ceres::QuaternionParameterization();
    problem.SetParameterization(&cameraRotation[0], cameraFitQuaternionParameterisation);
    ceres::Solver::Options solverOptions;
    solverOptions.linear_solver_type = ceres::SPARSE_SCHUR;
    solverOptions.num_threads = 1;
    solverOptions.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary solverSummary;
    //    ceres::Solve(solverOptions, &problem, &solverSummary);
    //std::cout << solverSummary.BriefReport() << "\n";
    //---------------------------------
    //    problem.SetParameterBlockVariable(&shape_coefficients[0]);
    //    problem.SetParameterBlockVariable(&blendshape_coefficients[0]);
    problem.SetParameterLowerBound(&shape_coefficients[0],0,-1500);
    problem.SetParameterUpperBound(&shape_coefficients[0],0,1500);
    problem.SetParameterLowerBound(&blendshape_coefficients[0],0,-1000);
    problem.SetParameterUpperBound(&blendshape_coefficients[0],0,1000);
    //        fitting::PriorCost *shapePrior=new fitting::PriorCost(num_coeffs_fitting, fx*100);
    fitting::PriorCost2 *shapePrior=new fitting::PriorCost2(num_coeffs_fitting,0.0015f);
    shapePrior->variance=FMFull.SV;
    ceres::CostFunction* shapePriorCost =
            new ceres::AutoDiffCostFunction<fitting::PriorCost2, num_coeffs_fitting /* num residuals */,
            num_coeffs_fitting /* shape-coeffs */>(
                shapePrior);
    problem.AddResidualBlock(shapePriorCost,/* new ceres::CauchyLoss(0.5)*/NULL, &shape_coefficients[0]);

    fitting::PriorCost2 *blendShapePrior=new fitting::PriorCost2(num_blendshapes,0.0003f);
    blendShapePrior->variance=FMFull.EV;
    ceres::CostFunction* blendshapesPriorCost =
            new ceres::AutoDiffCostFunction<fitting::PriorCost2, num_blendshapes /* num residuals */,
            num_blendshapes /* bs-coeffs */>(
                blendShapePrior);
    problem.AddResidualBlock(blendshapesPriorCost, NULL, &blendshape_coefficients[0]);
    ceres::Solve(solverOptions, &problem, &solverSummary);
    //std::cout << solverSummary.BriefReport() << "\n";



    Eigen::Quaternion<double> qd(cameraRotation[0],cameraRotation[1],cameraRotation[2],cameraRotation[3]);
    Eigen::Matrix<double, 3, 1> t(translation[0],translation[1],translation[2]);
    Eigen::Matrix<double,3,3> Rotation=qd.toRotationMatrix();
    //    std::cout << "Rotation:" << Rotation << std::endl;
    //    std::cout << "t:" << t << std::endl;
    Eigen::Matrix<float,3,3> Rf=Rotation.cast<float>();
    Eigen::Matrix<float,3,1> tf=t.cast<float>();
    params.R=Rf;
    params.tx=tf(0,0);
    params.ty=tf(1,0);
    params.tz=tf(2,0);
    params.s=1;
    MatD SXD= Eigen::Map<MatD>(shape_coefficients.data(), Eigen::Index(shape_coefficients.size() / 1), Eigen::Index(1));
    SX=SXD.cast<float>();
    //std::cout << "SX:" << SX << std::endl;
    MatD EXD= Eigen::Map<MatD>(blendshape_coefficients.data(), Eigen::Index(blendshape_coefficients.size() / 1), Eigen::Index(1));
    EX=EXD.cast<float>();
    //std::cout << "EX:" << EX << std::endl;
}
