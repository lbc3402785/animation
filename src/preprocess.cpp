#include "preprocess.h"

Preprocess::Preprocess()
{

}
void Preprocess::extractKeyShape(const MatF &SB, std::vector<int> &indexes, MatF &keySB)
{
    int n=indexes.size();
    int L=SB.cols();
    keySB.resize(n*3,L);
    for(int i=0;i<n;i++){
        int index=indexes[i];
        keySB.block(i*3,0,3,L)=SB.block(index*3,0,3,L);
    }
}

void Preprocess::extractKeyFace(const MatF& Face, std::vector<int> &indexes, MatF &keyFace)
{
    keyFace.resize(indexes.size(),3);
    for(int i=0;i<keyFace.rows();i++){
        keyFace.row(i)=Face.row(indexes[i]);
    }
}
void Preprocess::extractKeyFace(const std::vector<float>& Face, std::vector<int> &indexes, std::vector<float>  &keyFace)
{
    keyFace.resize(indexes.size()*3);
    for(int i=0;i<indexes.size();i++){
        keyFace[3*i+0]=Face[3*indexes[i]+0];
        keyFace[3*i+1]=Face[3*indexes[i]+1];
        keyFace[3*i+2]=Face[3*indexes[i]+2];
    }
}
void Preprocess::loadKeyIndex(string keyPath, std::vector<int> &keys, bool zero_begin)
{
    QSettings settings(QString::fromStdString(keyPath),QSettings::IniFormat);
    keys.clear();
    for(int i=1;i<=68;i++){
        QString key=QString::number(i);
        int value=settings.value(key,-1).value<int>();
        if(value==-1){
            std::cerr<<"invalid index!"<<std::endl<<std::flush;
        }else{
            //std::cerr<<"value:"<<value<<std::endl<<std::flush;
            if(!zero_begin){
                value-=1;
            }
            keys.push_back(value);
        }
    }
}

void Preprocess::loadKeyIndex2(string keyPath, std::vector<int> &keys, bool zeroBegin)
{
    std::ifstream in(keyPath);
    if(!in){
        std::cerr<<"read file:"<<keyPath<<" fail!"<<std::endl;
        exit(EXIT_FAILURE);
    }
    for(int i=0;i<68;i++){
        int value;
        in>>value;

        if(!zeroBegin)value-=1;
        keys.push_back(value);
    }
}

void Preprocess::extractBFMKeyModel(const FaceModel &BFMModel, std::vector<int> &indexes, FaceModel &keyModel)
{
    extractKeyShape(BFMModel.SB,indexes,keyModel.SB);
    extractKeyShape(BFMModel.EB,indexes,keyModel.EB);
    extractKeyFace(BFMModel.Face,indexes,keyModel.Face);
}
