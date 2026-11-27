#include "autonomymind/config.hpp"
#include <iostream>
int main(){try{auto c=autonomymind::CoreConfig::load_strict(FM_TEST_BASELINE);if(c.sha256.size()!=64)return 1;std::cout<<"PASS: strict config\n";return 0;}catch(...){return 1;}}
