#ifndef SETOPERATOR_H
#define SETOPERATOR_H

#include <iostream>
#include <vector>
#include <algorithm>
template<typename Type >
class SetOperator
{
public:
    static std::vector<Type> intersection( std::vector<Type>&A, std::vector<Type>&B);
    static std::vector<Type> setUnion( std::vector<Type>&A, std::vector<Type>&B);
    static std::vector<Type> difference( std::vector<Type>&A, std::vector<Type>&B);
    std::vector<Type> symmetricDifference( std::vector<Type>&A, std::vector<Type>&B);
};
template<typename Type>
std::vector<Type> SetOperator<Type>::intersection( std::vector<Type>&A, std::vector<Type>&B){
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
    std::vector<Type> result;
    std::set_intersection(A.begin(), A.end(),
    B.begin(), B.end(),
    std::back_inserter(result));
    return result;
}
template<typename Type>
std::vector<Type> SetOperator<Type>::setUnion( std::vector<Type>&A, std::vector<Type>&B){
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
    std::vector<Type> result;
    std::set_union(A.begin(), A.end(),
    B.begin(), B.end(),
    std::back_inserter(result));
    return result;
}
template<typename Type>
std::vector<Type> SetOperator<Type>::difference( std::vector<Type>&A, std::vector<Type>&B){
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
    std::vector<Type> result;
    std::set_difference(A.begin(), A.end(),
    B.begin(), B.end(),
    std::back_inserter(result));
    return result;
}
template<typename Type>
std::vector<Type> SetOperator<Type>::symmetricDifference( std::vector<Type>&A, std::vector<Type>&B){
    std::sort(A.begin(), A.end());
    std::sort(B.begin(), B.end());
    std::vector<Type> result;
    std::set_symmetric_difference(A.begin(), A.end(),
    B.begin(), B.end(),
    std::back_inserter(result));
    return result;
}
#endif // SETOPERATOR_H
