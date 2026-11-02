module demo_mboot(input wire enable,input wire [31:0] address);
 PH1P_LOGIC_MBOOT_V2 #(.MULTBOOT_ADDRSRC_SEL("DYNAMIC")) primitive_mboot(
  .multiboot_en_i(enable),.multiboot_code_i(8'hA5),.multiboot_addr_i(address));
endmodule
