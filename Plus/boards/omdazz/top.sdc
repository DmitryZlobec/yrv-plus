create_clock -period "50.0 MHz" [get_ports clk]

derive_clock_uncertainty

set_false_path -from [get_ports {key_sw[*]}] -to [all_clocks]
set_false_path -from rx -to [all_clocks]
set_false_path -from ir -to [all_clocks]

set_false_path -from * -to [get_ports {led[*]}]
set_false_path -from * -to [get_ports {abcdefgh[*]}]
set_false_path -from * -to [get_ports {digit[*]}]
set_false_path -from * -to buzzer
set_false_path -from * -to hsync
set_false_path -from * -to vsync
set_false_path -from * -to [get_ports {rgb[*]}]
set_false_path -from * -to tx
set_false_path -from * -to ser_rxd
set_false_path -from * -to ser_txd
#set_false_path -from * -to lpt_STROBE
#set_false_path -from * -to lpt_data
set_false_path -from * -to [get_ports {sram_a[*]}]
set_false_path -from * -to [get_ports {sram_io[*]}]
set_false_path -from * -to sram_cs2
set_false_path -from * -to sram_n_cs1
set_false_path -from * -to sram_n_oe
set_false_path -from * -to sram_n_we
