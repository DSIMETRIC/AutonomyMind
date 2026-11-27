#include "autonomymind/c_api.h"

#include <cstring>
#include <iostream>

int main() {
  if (fm_abi_version() != FM_ABI_VERSION) return 1;
  if (fm_raw_input_struct_size() != sizeof(fm_raw_input)) return 2;
  if (fm_output_struct_size() != sizeof(fm_output)) return 3;

  char error[256]{};
  fm_core_handle core = fm_core_create(FM_TEST_CONFIG, error, sizeof(error));
  if (!core) return 4;

  fm_raw_input input{};
  fm_output output{};
  input.abi_version = FM_ABI_VERSION;
  input.struct_size = sizeof(input);
  output.abi_version = FM_ABI_VERSION;
  output.struct_size = sizeof(output);
  input.armed = 1;
  input.nav_a_valid = 1;
  input.nav_b_valid = 1;

  if (fm_core_step(core, &input, &output, error, sizeof(error)) != 0) {
    fm_core_destroy(core);
    return 5;
  }

  input.abi_version = FM_ABI_VERSION + 1;
  if (fm_core_step(core, &input, &output, error, sizeof(error)) != -3) {
    fm_core_destroy(core);
    return 6;
  }

  input.abi_version = FM_ABI_VERSION;
  input.struct_size = sizeof(input) - 1;
  if (fm_core_step(core, &input, &output, error, sizeof(error)) != -3) {
    fm_core_destroy(core);
    return 7;
  }

  input.struct_size = sizeof(input);
  output.struct_size = sizeof(output) - 1;
  if (fm_core_step(core, &input, &output, error, sizeof(error)) != -4) {
    fm_core_destroy(core);
    return 8;
  }

  fm_core_destroy(core);
  std::cout << "PASS: C ABI version and struct-size hardening\n";
  return 0;
}
