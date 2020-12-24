#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "src/facemodel.h"
#include "src/Dlib.h"
#include <QSettings>
#include "util/eigenfunctions.h"
class Preprocess
{
public:
    Preprocess();
    static void extractKeyShape(const MatF &SB, std::vector<int> &indexes, MatF &keySB);
    static void extractKeyFace(const MatF& Face, std::vector<int> &indexes, MatF &keyFace);
    static void extractKeyFace(const std::vector<float>& Face, std::vector<int> &indexes, std::vector<float>  &keyFace);
    static void loadKeyIndex(std::string keyPath,std::vector<int>& keys,bool zero_begin=true);
    static void loadKeyIndex2(std::string keyPath,std::vector<int>& keys,bool zeroBegin=true);
    static void extractBFMKeyModel(const FaceModel &BFMModel, std::vector<int> &indexes, FaceModel &keyModel);
};

#endif // PREPROCESS_H
