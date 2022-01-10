#include <iostream>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

int main()
{
    using namespace boost::numeric::ublas;

    int jmbag[] = {0, 0, 3, 6, 5, 1, 7, 9, 4, 6};

    //a
    matrix<double> m1(5, 5);
    for (unsigned i = 0; i < m1.size1(); ++i)
        for (unsigned j = 0; j < m1.size2(); ++j)
            m1(i, j) = jmbag[j];
    std::cout << m1 << std::endl;

    identity_matrix<double> im(5);
    matrix<double> m2(5, 5);
    for (unsigned i = 0; i < m2.size1(); ++i)
        for (unsigned j = 0; j < m2.size2(); ++j)
            m2(i, j) = m1(i, j) + im(i, j);
    std::cout << m2 << std::endl;

    vector<double> v(5);
    for (unsigned i = 0; i < v.size(); ++i)
        v(i) = jmbag[i + 5];
    std::cout << v << std::endl;


    //b
    std::cout << prod(m2,v) << std::endl;

    //c
    matrix<double> v_T(5, 1);
    for (unsigned i = 0; i < v_T.size1(); ++i)
        for (unsigned j = 0; j < v_T.size2(); ++j)
            v_T(i, j) = v(i);
    std::cout << v_T << std::endl;      
    std::cout << prod(v, v_T) << std::endl;

    //d
    std::cout << m1 + m2 << std::endl;

    //e
    matrix<double> m2_copy(5, 5);
    m2_copy = m2;

    permutation_matrix<double> pm(m2_copy.size1());

    matrix<double> inverse(5, 5);
    for (unsigned i = 0; i < inverse.size1(); ++i)
        for (unsigned j = 0; j < inverse.size2(); ++j)
            if(i==j) inverse(i, j) = 1;

    int res = lu_factorize(m2_copy, pm);
    lu_substitute(m2_copy, pm, inverse);

    std::cout << inverse << std::endl;

    return 0;
}