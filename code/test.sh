rm *.o
rm main*
g++ -c KalmanFilter/Kalman.cpp
g++ -c test.cpp
g++ Kalman.o test.o -o main
./main.exe
rm *.o
rm main*