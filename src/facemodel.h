#ifndef FACEMODEL_H
#define FACEMODEL_H
#include "NumpyUtil.h"

class FaceModel
{
public:
    MatF SM;
    MatF SB;
    MatF SV;
    MatF SN;

    MatF EM;
    MatF EB;
    MatF EV;
    MatF EN;

    MatF CM;
    MatF CB;
    MatF CV;
    MatF CN;

    MatI TRI;
    MatI TRIUV;
    MatI TRI_FACE;
    MatI TRIUV_FACE;
    MatI TRI_HEAD;
    MatI TRIUV_HEAD;
    MatI TRI_BODY;
    MatI TRIUV_BODY;
    MatI Ef;
    MatI Ev;

    MatI HEAD_OFFSET;
    MatI BODY_OFFSET;
    MatF UV;
    MatF originFace;
    MatF Face;
    MatF Color;
    MatF GeneratedFace;
    Matrix<float, 1, Eigen::Dynamic> Mean;

    FaceModel()
    {

    }
    void Initialize(string file, bool LoadEdge,bool LoadColor=false);
    MatF Generate(MatF SX, MatF EX);
};

#endif // FACEMODEL_H
