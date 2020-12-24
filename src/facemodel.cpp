#include "facemodel.h"
void FaceModel::Initialize(string file, bool LoadEdge,bool LoadColor)
{
    cnpy::npz_t npz = cnpy::npz_load(file);

    SM = ToEigen(npz["SM"]);
    SB = ToEigen(npz["SB"]);



    EM = ToEigen(npz["EM"]);
    EB = ToEigen(npz["EB"]);

    try {
        UV = ToEigen(npz["UV"]);
        std::cout<<"UV"<<std::endl;
    } catch (...) {

    }


    MatF FaceFlat = SM + EM; // 204 * 1
    Face = Reshape(FaceFlat, 3);
    originFace=Face;
    try {
        TRI = ToEigenInt(npz["TRI"]);
        std::cout<<"TRI"<<std::endl;
    } catch (...) {

    }
    try
    {
        TRIUV = ToEigenInt(npz["TRIUV"]);
        std::cout<<"TRIUV"<<std::endl;
    }
    catch (...)
    {
        TRIUV = TRI;
    }
    try {
        SV = ToEigen(npz["SV"]);
        std::cout<<"SV"<<std::endl;
    } catch (...) {
        SV=MatF::Ones(199,1);
    }
    try {
        EV = ToEigen(npz["EV"]);
        std::cout<<"EV"<<std::endl;
    } catch (...) {
        EV=MatF::Ones(100,1);
    }

    if (LoadEdge)
    {


        try {
            std::cout<<"------------------------------"<<std::endl;
            HEAD_OFFSET=ToEigenInt(npz["HEAD_OFFSET"]);
            std::cout<<"HEAD_OFFSET"<<std::endl;
            BODY_OFFSET=ToEigenInt(npz["BODY_OFFSET"]);

        } catch (...) {

        }


        try {
            TRI_FACE= ToEigenInt(npz["TRI_FACE"]);
            std::cout<<"TRI_FACE"<<std::endl;
        } catch (...) {

        }
        try {
            TRI_HEAD= ToEigenInt(npz["TRI_HEAD"]);
            std::cout<<"TRI_HEAD"<<std::endl;
        } catch (...) {

        }
        try {
            TRI_BODY= ToEigenInt(npz["TRI_BODY"]);
            std::cout<<"TRI_BODY"<<std::endl;
        } catch (...) {

        }
        try {
            TRIUV_FACE= ToEigenInt(npz["TRIUV_FACE"]);
            std::cout<<"TRIUV_FACE"<<std::endl;
        } catch (...) {

        }
        try {
            TRIUV_HEAD= ToEigenInt(npz["TRIUV_HEAD"]);
            std::cout<<"TRIUV_HEAD"<<std::endl;
        } catch (...) {

        }
        try {
            TRIUV_BODY= ToEigenInt(npz["TRIUV_BODY"]);
            std::cout<<"TRIUV_BODY"<<std::endl;
        } catch (...) {

        }



        try
        {
            Ef = ToEigenInt(npz["Ef"]);
            Ev = ToEigenInt(npz["Ev"]);
        }
        catch (...)
        {
        }
    }
    if(LoadColor){

        CM = ToEigen(npz["CM"]);
        CB = ToEigen(npz["CB"]);

        CV = ToEigen(npz["CV"]);
        CN = ToEigen(npz["CN"]);
        MatF ColorFlat=CM;
        Color = Reshape(ColorFlat,3);
        std::cout<<"Color"<<std::endl;
    }
}


MatF FaceModel::Generate(MatF SX, MatF EX)
{
    MatF FaceS = SB * SX;
    MatF S = Reshape(FaceS, 3);

    MatF FaceE = EB * EX;
    MatF E = Reshape(FaceE, 3);

    GeneratedFace =  Face + S + E;
    return GeneratedFace;
}
