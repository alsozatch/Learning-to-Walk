#include <iostream>
using namespace std;

int main() {
    string options = "";
    int counter = 0;
    for(double i = -0.1; i <=0.1; i += 0.1) {
        for(double j = -0.1; j <=0.1; j += 0.1) {
            for(double k = -0.1; k <=0.1; k += 0.1) {
                for(double l = -0.1; l <=0.1; l += 0.1) {
                    options += ("[" + to_string(i) + "," + to_string(j) + "," + to_string(k) + "," + to_string(l) + "], ");
                    counter++;
                }
            }
        }
    }
    cout << options;
    cout << counter;
    return 0;
}
