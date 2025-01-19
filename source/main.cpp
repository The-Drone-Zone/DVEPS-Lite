#include <cstdlib>
#include <iostream>

int     main( int argc, const char* argv[]) {
    std::cout << "Hello, World!" << std::endl;

    if (argc != 3) {
        
        std::cerr << "Usage: " << argv[0] << " <num1> <num2>" << std::endl;
            return 1;
    }

    int num1 = std::atoi(argv[1]);
    int num2 = std::atoi(argv[2]);

    std::cout << "The sum of " << num1 << " and " << num2 << " is " << (num1 + num2) << std::endl;

    return 0;
}
