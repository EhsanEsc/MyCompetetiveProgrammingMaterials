if [[ $# -eq 1 ]]; then
    g++ -std=c++11 -Wall -Wextra $1.cpp -o a.out && ./a.out < test/in.txt > test/oo.txt 2> test/cerr.txt
elif [[ $# -eq 2 ]]; then
    g++ -std=c++11 -Wall -Wextra $1.cpp -o a.out && ./a.out < test/$2.txt > test/oo.txt 2> test/cerr.txt
else
    echo "Wrong number of Arguments!"
fi

if(diff -qb test/oo.txt test/out.txt);
then
    echo "Right!"
else
    echo "Wrong!"
fi
