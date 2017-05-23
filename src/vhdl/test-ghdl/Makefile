
bensvic2:	../debugtools.vhdl ../bensvic2.vhdl bensvic2_tb.vhdl bensvic2_tb.gtkw
	ghdl -a ../debugtools.vhdl ../bensvic2.vhdl bensvic2_tb.vhdl
	ghdl -e bensvic2_tb
	ghdl -r bensvic2_tb --stop-time=25ms --vcd=bensvic2_tb.vcd
	gtkwave bensvic2_tb.vcd bensvic2_tb.gtkw

std_fifo:	../std_fifo.vhdl std_fifo_tb.vhdl
	ghdl -a ../std_fifo.vhdl std_fifo_tb.vhdl
	ghdl -e std_fifo_tb
	ghdl -r std_fifo_tb --stop-time=11us --vcd=std_fifo_tb.vcd
	gtkwave std_fifo_tb.vcd std_fifo_tb.gtkw

clean:
	rm -f *.vcd
	rm -f work*.cf
