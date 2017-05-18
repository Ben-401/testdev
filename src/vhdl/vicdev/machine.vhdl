--
-- Written by
--    Paul Gardner-Stephen <hld@c64.org>  2013-2014
--
-- *  This program is free software; you can redistribute it and/or modify
-- *  it under the terms of the GNU Lesser General Public License as
-- *  published by the Free Software Foundation; either version 3 of the
-- *  License, or (at your option) any later version.
-- *
-- *  This program is distributed in the hope that it will be useful,
-- *  but WITHOUT ANY WARRANTY; without even the implied warranty of
-- *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- *  GNU General Public License for more details.
-- *
-- *  You should have received a copy of the GNU Lesser General Public License
-- *  along with this program; if not, write to the Free Software
-- *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
-- *  02111-1307  USA.

----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    22:30:37 12/10/2013 
-- Design Name: 
-- Module Name:    container - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;


-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity machine is
  Port (
    sysclk : std_logic;
    reset_S : std_logic;
    reset_L : std_logic;
    pixelclock_en : std_logic;
    cpuioclock_en : std_logic;

--         pixelclock : STD_LOGIC;
--         pixelclock2x : STD_LOGIC;
--         cpuclock : std_logic;
----         clock50mhz : in std_logic;
--         ioclock : std_logic;
--         uartclock : std_logic;
			
--         btnCpuReset : in  STD_LOGIC;
			
         irq : in  STD_LOGIC;
        nmi : in  STD_LOGIC;

--         no_kickstart : in std_logic;
--
--         ddr_counter : in unsigned(7 downto 0);
--         ddr_state : in unsigned(7 downto 0);
         
         ----------------------------------------------------------------------
         -- VGA output
         ----------------------------------------------------------------------
         vsync : out  STD_LOGIC;
         hsync : out  STD_LOGIC;
         vgared : out  UNSIGNED (3 downto 0);
         vgagreen : out  UNSIGNED (3 downto 0);
         vgablue : out  UNSIGNED (3 downto 0);

         -------------------------------------------------------------------------
         -- Lines for the SDcard interface itself
         -------------------------------------------------------------------------
--         cs_bo : out std_logic;
--         sclk_o : out std_logic;
--         mosi_o : out std_logic;
--         miso_i : in  std_logic;
--
--         ---------------------------------------------------------------------------
--         -- Lines for other devices that we handle here
--         ---------------------------------------------------------------------------
--         aclMISO : in std_logic;
--         aclMOSI : out std_logic;
--         aclSS : out std_logic;
--         aclSCK : out std_logic;
--         aclInt1 : in std_logic;
--         aclInt2 : in std_logic;
--    
--         ampPWM : out std_logic;
--         ampSD : out std_logic;
--
--         micData : in std_logic;
--         micClk : out std_logic;
--         micLRSel : out std_logic;
--
--         tmpSDA : out std_logic;
--         tmpSCL : out std_logic;
--         tmpInt : in std_logic;
--         tmpCT : in std_logic;
--         
--         ---------------------------------------------------------------------------
--         -- IO lines to the ethernet controller
--         ---------------------------------------------------------------------------
--         eth_mdio : inout std_logic;
--         eth_mdc : out std_logic;
--         eth_reset : out std_logic;
--         eth_rxd : in unsigned(1 downto 0);
--         eth_txd : out unsigned(1 downto 0);
--         eth_txen : out std_logic;
--         eth_rxdv : in std_logic;
--         eth_rxer : in std_logic;
--         eth_interrupt : in std_logic;
--         
--         ----------------------------------------------------------------------
--         -- Flash RAM for holding config
--         ----------------------------------------------------------------------
--         QspiSCK : out std_logic;
--         QspiDB : inout std_logic_vector(3 downto 0);
--         QspiCSn : out std_logic;
--
--         fpga_temperature : in std_logic_vector(11 downto 0);
--         
--         ---------------------------------------------------------------------------
--         -- Interface to Slow RAM (128MB DDR2 RAM chip)
--         ---------------------------------------------------------------------------
--         slowram_addr_reflect : in std_logic_vector(26 downto 0);
--         slowram_datain_reflect : in std_logic_vector(7 downto 0);
--         slowram_addr : out std_logic_vector(26 downto 0);
--         slowram_we : out std_logic;
--         slowram_request_toggle : out std_logic;
--         slowram_done_toggle : in std_logic;
--         slowram_datain : out std_logic_vector(7 downto 0);
--         -- simple-dual-port cache RAM interface so that CPU doesn't have to read
--         -- data cross-clock
--         cache_address        : out std_logic_vector(8 downto 0);
--         cache_read_data      : in std_logic_vector(150 downto 0);   
--
--         ----------------------------------------------------------------------
--         -- PS/2 adapted USB keyboard & joystick connector.
--         -- (For using a keyrah adapter to connect to the keyboard.)
--         ----------------------------------------------------------------------
--         ps2data : in std_logic;
--         ps2clock : in std_logic;
--
--         ----------------------------------------------------------------------
--         -- PMOD interface for keyboard, joystick, expansion port etc board.
--         ----------------------------------------------------------------------
--         pmod_clock : in std_logic;
--         pmod_start_of_sequence : in std_logic;
--         pmod_data_in : in std_logic_vector(3 downto 0);
--         pmod_data_out : out std_logic_vector(1 downto 0);
--         pmoda : inout std_logic_vector(7 downto 0);
--
--         uart_rx : in std_logic;
--         uart_tx : out std_logic;
--    
--         ----------------------------------------------------------------------
--         -- Debug interfaces on Nexys4 board
--         ----------------------------------------------------------------------
         led : out std_logic_vector(15 downto 0);
         sw : in std_logic_vector(15 downto 0);
--         btn : in std_logic_vector(4 downto 0);
--
--         UART_TXD : out std_logic;
--         RsRx : in std_logic;
--         
         sseg_ca : out std_logic_vector(7 downto 0);
         sseg_an : out std_logic_vector(7 downto 0)
         );
end machine;

architecture Behavioral of machine is

  component bensvic2 is
    Port (
    sysclk        : in std_logic;
    reset_S       : in std_logic;
    reset_L       : in std_logic;
    pixelclock_en : in std_logic;
    cpuioclock_en : in std_logic;
    ----------------------------------------------------------------------
    -- VGA output
    ----------------------------------------------------------------------
    vsync : out  STD_LOGIC;
    hsync : out  STD_LOGIC;
    vgared : out  UNSIGNED (3 downto 0);
    vgagreen : out  UNSIGNED (3 downto 0);
    vgablue : out  UNSIGNED (3 downto 0)
      );
  end component;




--  component viciv is
--    Port (
--    sysclk        : in std_logic;
--	 reset2        : in std_logic;
--	 pixelclock_en : in std_logic;
--	 cpuioclock_en : in std_logic;
--    
----    ----------------------------------------------------------------------
----    -- dot clock
----    ----------------------------------------------------------------------
----    pixelclock : in  STD_LOGIC;
----    pixelclock2x : in std_logic;
----    ----------------------------------------------------------------------
----    -- CPU clock (used for chipram and fastio interfaces)
----    ----------------------------------------------------------------------
----    cpuclock : in std_logic;
----    ioclock : in std_logic;
--
--    -- CPU IRQ
--    irq : out std_logic;
--
----    reset : in std_logic;
--
--    -- Internal drive LED status for OSD
--    led : in std_logic;
--    motor : in std_logic;
--
--    -- Actual drive LED (including blink status) for keyboard
--    -- (the F011 does this on a real C65)
--    drive_led_out : out std_logic;
--
--    vicii_2mhz : out std_logic;
--    viciii_fast : out std_logic;
--    viciv_fast : out std_logic;     
--
--    xray_mode : in std_logic;
--    
--    ----------------------------------------------------------------------
--    -- VGA output
--    ----------------------------------------------------------------------
--    vsync : out  STD_LOGIC;
--    hsync : out  STD_LOGIC;
--    vgared : out  UNSIGNED (3 downto 0);
--    vgagreen : out  UNSIGNED (3 downto 0);
--    vgablue : out  UNSIGNED (3 downto 0);
--
--    pixel_stream_out : out unsigned (7 downto 0);
--    pixel_y : out unsigned (11 downto 0);
--    pixel_valid : out std_logic;
--    pixel_newframe : out std_logic;
--    pixel_newraster : out std_logic;
--    
--    ---------------------------------------------------------------------------
--    -- CPU Interface to ChipRAM in video controller (just 128KB for now)
--    ---------------------------------------------------------------------------
--    chipram_we : IN STD_LOGIC;
--    chipram_address : IN unsigned(16 DOWNTO 0);
--    chipram_datain : IN unsigned(7 DOWNTO 0);
--    
--    -----------------------------------------------------------------------------
--    -- FastIO interface for accessing video registers
--    -----------------------------------------------------------------------------
--    fastio_addr : in std_logic_vector(19 downto 0);
--    fastio_read : in std_logic;
--    fastio_write : in std_logic;
--    fastio_wdata : in std_logic_vector(7 downto 0);
--    fastio_rdata : out std_logic_vector(7 downto 0);
--    colour_ram_fastio_rdata : out std_logic_vector(7 downto 0);
--    colour_ram_cs : in std_logic;
--    charrom_write_cs : in std_logic;
--
--    viciii_iomode : out std_logic_vector(1 downto 0) := "11";
--
--    iomode_set : in std_logic_vector(1 downto 0);
--    iomode_set_toggle : in std_logic;
--
--    colourram_at_dc00 : out std_logic := '0';   
--    rom_at_e000 : out std_logic;
--    rom_at_c000 : out std_logic;
--    rom_at_a000 : out std_logic;
--    rom_at_8000 : out std_logic
--      );
--  end component;

  
  signal key_scancode : unsigned(15 downto 0);
  signal key_scancode_toggle : std_logic;

  signal xray_mode : std_logic;
  
  signal cpu_hypervisor_mode : std_logic;

  signal reg_isr_out : unsigned(7 downto 0);
  signal imask_ta_out : std_logic;
  
  signal cpu_leds : std_logic_vector(3 downto 0);
  
  signal viciii_iomode : std_logic_vector(1 downto 0);

  signal iomode_set : std_logic_vector(1 downto 0);
  signal iomode_set_toggle : std_logic;

  signal vicii_2mhz : std_logic;
  signal viciii_fast : std_logic;
  signal viciv_fast : std_logic;
  signal capslock_state : std_logic;
  signal speed_gate : std_logic;
  signal speed_gate_enable : std_logic;
  
  signal drive_led : std_logic;
  signal motor : std_logic;
  signal drive_led_out : std_logic;
  
  signal seg_led_data : unsigned(31 downto 0);

  signal reset_out : std_logic;
  signal reset_monitor : std_logic;
  -- Holds reset on for 8 cycles so that reset line entry is used on start up,
  -- instead of implicit startup state.
  signal power_on_reset : std_logic_vector(7 downto 0) := (others => '1');
  signal reset_combined : std_logic := '1';
  
  signal io_irq : std_logic;
  signal io_nmi : std_logic;
  signal vic_irq : std_logic;
  signal combinedirq : std_logic;
  signal combinednmi : std_logic;
  signal restore_nmi : std_logic;
  signal hyper_trap : std_logic := '1';
  signal hyper_trap_combined : std_logic := '1';
  signal monitor_hyper_trap : std_logic := '1';

  signal fastio_addr : std_logic_vector(19 downto 0);
  signal fastio_read : std_logic;
  signal fastio_write : std_logic;
  signal fastio_wdata : std_logic_vector(7 downto 0);
  signal fastio_rdata : std_logic_vector(7 downto 0);
  signal fastio_vic_rdata : std_logic_vector(7 downto 0);
  signal colour_ram_fastio_rdata : std_logic_vector(7 downto 0);
  signal sector_buffer_mapped : std_logic;

  signal chipram_we : STD_LOGIC;
  signal chipram_address : unsigned(16 DOWNTO 0);
  signal chipram_datain : unsigned(7 DOWNTO 0);
  
  signal rom_at_e000 : std_logic := '0';
  signal rom_at_c000 : std_logic := '0';
  signal rom_at_a000 : std_logic := '0';
  signal rom_at_8000 : std_logic := '0';

  signal colourram_at_dc00 : std_logic := '0';
  signal colour_ram_cs : std_logic := '0';
  signal charrom_write_cs : std_logic := '0';

  signal monitor_pc : unsigned(15 downto 0);
  signal monitor_hypervisor_mode : std_logic;
  signal monitor_ddr_ram_banking : std_logic;
  signal monitor_state : unsigned(15 downto 0);
  signal monitor_instruction : unsigned(7 downto 0);
  signal monitor_instructionpc : unsigned(15 downto 0);
  signal monitor_watch : unsigned(27 downto 0);
--  signal monitor_debug_memory_access : std_logic_vector(31 downto 0);
  signal monitor_proceed : std_logic;
  signal monitor_waitstates : unsigned(7 downto 0);
  signal monitor_request_reflected : std_logic;
  signal monitor_watch_match : std_logic;
  signal monitor_mem_address : unsigned(27 downto 0);
  signal monitor_mem_rdata : unsigned(7 downto 0);
  signal monitor_mem_wdata : unsigned(7 downto 0);
  signal monitor_map_offset_low : unsigned(11 downto 0);
  signal monitor_map_offset_high : unsigned(11 downto 0);
  signal monitor_map_enables_low : std_logic_vector(3 downto 0);
  signal monitor_map_enables_high : std_logic_vector(3 downto 0);   
  signal monitor_mem_read : std_logic;
  signal monitor_mem_write : std_logic;
  signal monitor_mem_setpc : std_logic;
  signal monitor_mem_attention_request : std_logic;
  signal monitor_mem_attention_granted : std_logic;
  signal monitor_mem_stage_trace_mode : std_logic;
  signal monitor_mem_trace_mode : std_logic;
  signal monitor_mem_trace_toggle : std_logic;
  signal monitor_char : unsigned(7 downto 0);
  signal monitor_char_toggle : std_logic;
  
  signal monitor_a : unsigned(7 downto 0);
  signal monitor_b : unsigned(7 downto 0);
  signal monitor_interrupt_inhibit : std_logic;
  signal monitor_x : unsigned(7 downto 0);
  signal monitor_y : unsigned(7 downto 0);
  signal monitor_z : unsigned(7 downto 0);
  signal monitor_sp : unsigned(15 downto 0);
  signal monitor_p : unsigned(7 downto 0);
  signal monitor_opcode : unsigned(7 downto 0);
  signal monitor_ibytes : std_logic_vector(3 downto 0);
  signal monitor_arg1 : unsigned(7 downto 0);
  signal monitor_arg2 : unsigned(7 downto 0);

  signal cpuis6502 : std_logic;
  signal cpuspeed : unsigned(7 downto 0);

  
  signal segled_counter : unsigned(17 downto 0) := (others => '0');

  -- Clock running as close as possible to 17.734475 MHz / 18 = 985248Hz
  -- Our pixel clock is 192MHz.  195 ticks gives 984615Hz for NTSC.
  -- 188 ticks at 192MHz gives 1021276Hz, which is pretty close for PAL.
  -- But dotclock is really 193.75MHz, so adjust accordingly.
  -- Then divide by 2 again, since the loop toggles phi0.
  signal phi0 : std_logic := '0';
  constant phi0_divisor : integer := 95;
  signal phi0_counter : integer range 0 to phi0_divisor;

--  signal pixel_stream : unsigned (7 downto 0);
--  signal pixel_y : unsigned (11 downto 0);
--  signal pixel_valid : std_logic;
--  signal pixel_newframe : std_logic;
--  signal pixel_newraster : std_logic;

  signal farcallstack_we : std_logic;
  signal farcallstack_addr : std_logic_vector(8 downto 0);
  signal farcallstack_din : std_logic_vector(63 downto 0);
  signal farcallstack_dout : std_logic_vector(63 downto 0);


begin

  ----------------------------------------------------------------------------
  -- IRQ & NMI: If either the hardware buttons on the FPGA board or an IO
  -- device via the IOmapper pull an interrupt line down, then trigger an
  -- interrupt.
  -----------------------------------------------------------------------------
  process(irq, nmi, restore_nmi,
          io_irq,
			 vic_irq,
			 io_nmi,
			 sw,
			 reset_out,
			 reset_S, reset_L,
          power_on_reset,
			 reset_monitor,
			 hyper_trap,
			 monitor_hyper_trap)
  begin
  
    -- XXX Allow switch 0 to mask IRQs
    combinedirq <= (irq and io_irq and vic_irq)     or sw(0);
    combinednmi <= (nmi and io_nmi and restore_nmi) or sw(14);
	 
	 reset_combined <= reset_S;
	 
--    if reset2='0' then
--      report "reset asserted via btnCpuReset";
--      reset_combined <= '0';
--    elsif reset_out='0' then
--      report "reset asserted via reset_out";
--      reset_combined <= '0';
--    elsif power_on_reset(0)='0' then
--      report "reset asserted via power_on_reset(0)";
--      reset_combined <= '0';
--    elsif reset_monitor='0' then
--      report "reset asserted via reset_monitor";
--      reset_combined <= '0';
--    else
--      reset_combined <= '1';
--    end if;

    hyper_trap_combined <= hyper_trap and monitor_hyper_trap;
    
    -- report "btnCpuReset = " & std_logic'image(btnCpuReset) & ", reset_out = " & std_logic'image(reset_out) & ", sw(15) = " & std_logic'image(sw(15)) severity note;
    -- report "reset_combined = " & std_logic'image(reset_combined) severity note;
  end process;

  hyper_trap				<= sw(0);
  monitor_hyper_trap		<= sw(0);
  colour_ram_cs			<= sw(0);
  charrom_write_cs		<= sw(0);
  
  viciii_iomode         <= sw(1) & sw(0);
  vicii_2mhz				<= sw(0);
  viciii_fast				<= sw(0);
  viciv_fast				<= sw(0);
  vic_irq					<= sw(0);
  colour_ram_fastio_rdata <= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);

  cpu_hypervisor_mode	<= sw(0);
  iomode_set(1)			<= sw(0);
  iomode_set(0)			<= sw(0);
  iomode_set_toggle		<= sw(0);
  speed_gate				<= sw(0);
  speed_gate_enable		<= sw(0);
  drive_led					<= sw(0);
  motor						<= sw(0);
  io_irq						<= sw(0);
  io_nmi						<= sw(0);
  restore_nmi				<= sw(0);

  rom_at_e000				<= sw(0);
  rom_at_c000				<= sw(0);
  rom_at_a000				<= sw(0);
  rom_at_8000				<= sw(0);
  colourram_at_dc00		<= sw(0);

  fastio_addr				<= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0) &
									sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0) &
								   sw(7) & sw(6) & sw(5) & sw(4);
									
  fastio_read				<= sw(0);
  fastio_write				<= sw(0);
  fastio_wdata				<= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);
  fastio_rdata				<= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);
  chipram_we				<= sw(0);
  chipram_address			<= sw(7) &
                         sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0) &
                         sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);
  chipram_datain			<= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);

  cpuspeed					<= sw(7) & sw(6) & sw(5) & sw(4) & sw(3) & sw(2) & sw(1) & sw(0);
  cpuis6502					<= sw(1);
		  

-- ######## ########
-- ######## ########

      led(0) <= sw(0) or irq; -- input to machine
      led(1) <= sw(1) or nmi; -- input to machine
      led(2) <= sw(2) or combinedirq; -- internal to machine
      led(3) <= sw(3) or combinednmi; -- internal to machine
      led(4) <= sw(4) or io_irq; -- internal to machine
      led(5) <= sw(5) or io_nmi; -- internal to machine

      led(6) <= sw(6); -- or
--                      pixel_stream(7) or pixel_stream(6) or pixel_stream(5) or pixel_stream(4) or
--							 pixel_stream(3) or pixel_stream(2) or pixel_stream(1) or pixel_stream(0) or
--							 pixel_y(11) or pixel_y(10) or pixel_y(9) or pixel_y(8) or
--							 pixel_y(7) or pixel_y(6) or pixel_y(5) or pixel_y(4) or
--							 pixel_y(3) or pixel_y(2) or pixel_y(1) or pixel_y(0) or
--							 pixel_valid or pixel_newframe or pixel_newraster;
		
      led(7) <= sw(7) or
		fastio_addr(19) or fastio_addr(18) or fastio_addr(17) or fastio_addr(16) or 
		fastio_addr(15) or fastio_addr(14) or fastio_addr(13) or fastio_addr(12) or 
		fastio_addr(11) or fastio_addr(10) or fastio_addr(9) or fastio_addr(8) or 
		fastio_addr(7) or fastio_addr(6) or fastio_addr(5) or fastio_addr(4) or 
		fastio_addr(3) or fastio_addr(2) or fastio_addr(1) or fastio_addr(0) or 
		
		fastio_read or
		fastio_write or

		fastio_rdata(7) or fastio_rdata(6) or fastio_rdata(5) or fastio_rdata(4) or
		fastio_rdata(3) or fastio_rdata(2) or fastio_rdata(1) or fastio_rdata(0) or

		fastio_wdata(7) or fastio_wdata(6) or fastio_wdata(5) or fastio_wdata(4) or
		fastio_wdata(3) or fastio_wdata(2) or fastio_wdata(1) or fastio_wdata(0) or

		colour_ram_fastio_rdata(7) or colour_ram_fastio_rdata(6) or
		colour_ram_fastio_rdata(5) or colour_ram_fastio_rdata(4) or
		colour_ram_fastio_rdata(3) or colour_ram_fastio_rdata(2) or
		colour_ram_fastio_rdata(1) or colour_ram_fastio_rdata(0);
		
      led(8) <= sw(8) or viciii_iomode(1) or viciii_iomode(0) or
		          vicii_2mhz or viciii_fast or viciv_fast or
					 
					 chipram_we or

					 chipram_address(16) or 
					 chipram_address(15) or chipram_address(14) or chipram_address(13) or chipram_address(12) or 
					 chipram_address(11) or chipram_address(10) or chipram_address(9) or chipram_address(8) or 
					 chipram_address(7) or chipram_address(6) or chipram_address(5) or chipram_address(4) or 
					 chipram_address(3) or chipram_address(2) or chipram_address(1) or chipram_address(0) or 

					 chipram_datain(7) or chipram_datain(6) or chipram_datain(5) or chipram_datain(4) or 
					 chipram_datain(3) or chipram_datain(2) or chipram_datain(1) or chipram_datain(0);

			 
      led(9) <= sw(9) or rom_at_e000 or
		                   rom_at_c000 or
								 rom_at_a000 or
								 rom_at_8000 or
								 colourram_at_dc00 or
								 colour_ram_cs or
								 charrom_write_cs or
								 iomode_set_toggle or
								 drive_led or
								 motor or
								 xray_mode;

      led(10) <= sw(10) or cpu_hypervisor_mode; -- internal to machine
      led(11) <= sw(11) or hyper_trap; -- internal to machine
      led(12) <= sw(12) or hyper_trap_combined; -- internal to machine
      led(13) <= sw(13) or monitor_hyper_trap; -- internal to machine
      led(14) <= sw(14) or speed_gate; -- internal to machine
      led(15) <= sw(15) or speed_gate_enable; -- internal to machine

      xray_mode <= sw(1);

-- ######## ########
-- ######## ########
  
  process(sysclk, reset_S) --pixelclock,ioclock)
    variable digit : std_logic_vector(3 downto 0);
  begin
  
    if rising_edge(sysclk) then --ioclock) then
--      -- Hold reset low for a while when we first turn on
----      report "power_on_reset(0) = " & std_logic'image(power_on_reset(0)) severity note;
--      power_on_reset(7) <= '1';
--      power_on_reset(6 downto 0) <= power_on_reset(7 downto 1);

      segled_counter <= segled_counter + 1;

      sseg_an <= (others => '1');
      sseg_an(to_integer(segled_counter(17 downto 15))) <= '0';

      if segled_counter(17 downto 15)=0 then
        digit := std_logic_vector(seg_led_data(3 downto 0));
      elsif segled_counter(17 downto 15)=1 then
        digit := std_logic_vector(seg_led_data(7 downto 4));
      elsif segled_counter(17 downto 15)=2 then
        digit := std_logic_vector(seg_led_data(11 downto 8));
      elsif segled_counter(17 downto 15)=3 then
        digit := std_logic_vector(seg_led_data(15 downto 12));
      elsif segled_counter(17 downto 15)=4 then
        --digit := std_logic_vector(seg_led_data(19 downto 16));
        digit := (others => '0');
      elsif segled_counter(17 downto 15)=5 then
        --digit := std_logic_vector(seg_led_data(23 downto 20));
        digit := (others => '0');
      elsif segled_counter(17 downto 15)=6 then
        digit := std_logic_vector(seg_led_data(27 downto 24));
      elsif segled_counter(17 downto 15)=7 then
        digit := std_logic_vector(seg_led_data(31 downto 28));
      end if;
      
      if cpuis6502 = '1' then
        seg_led_data(15 downto 0) <= x"6510";
      else
        seg_led_data(15 downto 0) <= x"4502";
      end if;
      seg_led_data(31 downto 24) <= cpuspeed;
      
      -- segments are:
      -- 7 - decimal point
      -- 6 - middle
      -- 5 - upper left
      -- 4 - lower left
      -- 3 - bottom
      -- 2 - lower right
      -- 1 - upper right
      -- 0 - top
      case digit is
        when x"0" => sseg_ca   <= "11000000";
        when x"1" => sseg_ca   <= "11111001";
        when x"2" => sseg_ca   <= "10100100";
        when x"3" => sseg_ca   <= "10110000";
        when x"4" => sseg_ca   <= "10011001";
        when x"5" => sseg_ca   <= "10010010";
        when x"6" => sseg_ca   <= "10000010";
        when x"7" => sseg_ca   <= "11111000";
        when x"8" => sseg_ca   <= "10000000";
        when x"9" => sseg_ca   <= "10010000";
        when x"A" => sseg_ca   <= "10001000";
        when x"B" => sseg_ca   <= "10000011";
        when x"C" => sseg_ca   <= "11000110";
        when x"D" => sseg_ca   <= "10100001";
        when x"E" => sseg_ca   <= "10000110";
        when x"F" => sseg_ca   <= "10001110";
        when others => sseg_ca <= "10100001";
      end case; 
      

    end if;

--    if rising_edge(pixelclock) then
--      -- Work out phi0 frequency for CIA timers
--      if phi0_counter=phi0_divisor then
--        phi0 <= not phi0;
--        phi0_counter <= 0;
--      else
--        phi0_counter <= phi0_counter + 1;
--      end if;
--    end if;

  end process;
  


  viciv0: bensvic2
    port map (
	   sysclk => sysclk,
		reset_S => reset_S,
		reset_L => reset_L,
		pixelclock_en => pixelclock_en,
		cpuioclock_en => cpuioclock_en,
      vsync           => vsync,
      hsync           => hsync,
      vgared          => vgared,
      vgagreen        => vgagreen,
      vgablue         => vgablue 
      );

--  viciv0: viciv
--    port map (
--	   sysclk => sysclk,
--		reset2 => reset_combined,
--		pixelclock_en => pixelclock_en,
--		cpuioclock_en => cpuioclock_en,	
--		
----      pixelclock      => pixelclock,
----      pixelclock2x      => pixelclock2x,
----      cpuclock        => cpuclock,
----      ioclock        => ioclock,
--
--      irq             => vic_irq,
----      reset           => reset_combined,
--
--      led => drive_led,
--      motor => motor,
--      drive_led_out => drive_led_out,
--
--      xray_mode => xray_mode,
--      
--      vsync           => vsync,
--      hsync           => hsync,
--      vgared          => vgared,
--      vgagreen        => vgagreen,
--      vgablue         => vgablue,
--
--      pixel_stream_out => pixel_stream,
--      pixel_y => pixel_y,
--      pixel_valid => pixel_valid,
--      pixel_newframe => pixel_newframe,
--      pixel_newraster => pixel_newraster,
--      
--      chipram_we => chipram_we,
--      chipram_address => chipram_address,
--      chipram_datain => chipram_datain,
--      colour_ram_fastio_rdata => colour_ram_fastio_rdata,
--      colour_ram_cs => colour_ram_cs,
--      charrom_write_cs => charrom_write_cs,
--
--      fastio_addr     => fastio_addr,
--      fastio_read     => fastio_read,
--      fastio_write    => fastio_write,
--      fastio_wdata    => fastio_wdata,
--      fastio_rdata    => fastio_vic_rdata,
--    
--      viciii_iomode => viciii_iomode,
--      iomode_set_toggle => iomode_set_toggle,
--      iomode_set => iomode_set,
--      vicii_2mhz => vicii_2mhz,
--      viciii_fast => viciii_fast,
--      viciv_fast => viciv_fast,
--    
--      colourram_at_dc00 => colourram_at_dc00,
--      rom_at_e000 => rom_at_e000,
--      rom_at_c000 => rom_at_c000,
--      rom_at_a000 => rom_at_a000,
--      rom_at_8000 => rom_at_8000      
--      );

  
 
end Behavioral;

