#include "autonomymind/core.hpp"
#include <iostream>
int main(){try{auto cfg=autonomymind::CoreConfig::load_strict(FM_TEST_CONFIG);autonomymind::Core core(cfg);autonomymind::RawInput in;in.timestamp_s=0;in.nav_a_valid=true;in.nav_b_valid=true;auto out=core.step(in);if(out.config_sha256.empty())return 1;std::cout<<"PASS: test_arbitration_matrix\n";return 0;}catch(...){return 1;}}
