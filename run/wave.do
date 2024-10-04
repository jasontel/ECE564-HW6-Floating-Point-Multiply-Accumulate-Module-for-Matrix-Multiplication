onerror {resume}

log -r /tb_top/dut/*
add wave -r /tb_top/dut/*

quietly WaveActivateNextPane {} 0
add wave -noupdate -color Cyan -itemcolor Cyan /tb_top/dut/dut__tb__sram_input_write_enable
add wave -noupdate -color Cyan -itemcolor Cyan -radix decimal /tb_top/dut/dut__tb__sram_input_read_address
add wave -noupdate -color Cyan -itemcolor Cyan -radix float32 /tb_top/dut/tb__dut__sram_input_read_data
add wave -noupdate -color Pink -itemcolor Pink /tb_top/dut/dut__tb__sram_weight_write_enable
add wave -noupdate -color Pink -itemcolor Pink /tb_top/dut/dut__tb__sram_weight_read_address
add wave -noupdate -color Pink -itemcolor Pink -radix float32 /tb_top/dut/tb__dut__sram_weight_read_data
add wave -noupdate /tb_top/dut/dut__tb__sram_result_write_enable
add wave -noupdate -radix decimal /tb_top/dut/dut__tb__sram_result_write_address
add wave -noupdate -radix float32 /tb_top/dut/dut__tb__sram_result_write_data
add wave -noupdate /tb_top/dut/dut__tb__sram_result_read_address
add wave -noupdate /tb_top/dut/tb__dut__sram_result_read_data
add wave -noupdate -divider {Control signals}
add wave -noupdate -radix float32 /tb_top/dut/accum_result
add wave -noupdate /tb_top/dut/compute_phase
add wave -noupdate /tb_top/dut/compute_complete
add wave -noupdate -divider FP_MAC
add wave -noupdate -radix float32 /tb_top/dut/FP_MAC/inst_a
add wave -noupdate -radix float32 /tb_top/dut/FP_MAC/inst_b
add wave -noupdate -radix float32 /tb_top/dut/FP_MAC/inst_c
add wave -noupdate /tb_top/dut/FP_MAC/inst_rnd
add wave -noupdate -radix float32 /tb_top/dut/FP_MAC/z_inst
add wave -noupdate /tb_top/dut/FP_MAC/status_inst
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {2375 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 317
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {2579 ns} {3483 ns}
