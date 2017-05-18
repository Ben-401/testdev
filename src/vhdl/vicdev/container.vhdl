----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
library UNISIM;
use UNISIM.VComponents.all;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

entity container is
  Port ( clk_in : STD_LOGIC;         
         btnCpuReset_in : in  STD_LOGIC; -- normally high, active low
--         irq : in  STD_LOGIC;
--         nmi : in  STD_LOGIC;
         ja1_out : out std_logic;
         ja2_out : out std_logic;
         ja3_out : out std_logic;
         ja4_out : out std_logic;

         ja7_out : out std_logic;
         ja8_out : out std_logic;
         ja9_out : out std_logic;
         jaa_out : out std_logic;
			
         ----------------------------------------------------------------------
         -- VGA output
         ----------------------------------------------------------------------
-- container
         vsync : out  STD_LOGIC;
         hsync : out  STD_LOGIC;
         vgared : out  UNSIGNED (3 downto 0);
         vgagreen : out  UNSIGNED (3 downto 0);
         vgablue : out  UNSIGNED (3 downto 0);

         


-- container
         ----------------------------------------------------------------------
-- container
-- container
-- container
         --led   : out std_logic_vector(15 downto 0);
         led_out : out std_logic_vector(15 downto 0);
	 
         --sw    : in std_logic_vector(15 downto 0);
         sw_in0  : in std_logic;
         sw_in1  : in std_logic;
         sw_in2  : in std_logic;
         sw_in3  : in std_logic;
         sw_in4  : in std_logic;
         sw_in5  : in std_logic;
         sw_in6  : in std_logic;
         sw_in7  : in std_logic;
         sw_in8  : in std_logic;
         sw_in9  : in std_logic;
         sw_inA  : in std_logic;
         sw_inB  : in std_logic;
         sw_inC  : in std_logic;
         sw_inD  : in std_logic;
         sw_inE  : in std_logic;
         sw_inF  : in std_logic;
			
--         btn_in : in std_logic_vector(4 downto 0);
			
-- container

--         UART_TXD : out std_logic;
--         RsRx : in std_logic;
         
         sseg_ca : out std_logic_vector(7 downto 0);
         sseg_an : out std_logic_vector(7 downto 0)
         );
end container;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

architecture Behavioral of container is

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  component clkgen is
    Port (
      clk_in : STD_LOGIC;         
      reset_ext_in : in  STD_LOGIC;
         
      clk0_out : out std_logic;
      clk1_out : out std_logic;
      locked_out : out std_logic;
      dbg_rstA : out std_logic;
      dbg_rstB : out std_logic
    );
  end component;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

 
--  signal pixelclock : std_logic;
--  signal pixelclock2x : std_logic;
--  signal cpuclock : std_logic;
  
  signal sw_int :    std_logic_vector(15 downto 0);
--  signal led_int :    std_logic_vector(15 downto 0);

  signal reset_delayed : std_logic_vector(7 downto 0);

  signal clk100buf : std_logic;

--  signal clock100mhz : std_logic; -- for LAN (TBC)
  signal clock387mhz : std_logic; -- primary clock is 387.5mhz
  signal clkdiv : unsigned(2 downto 0); -- counter for clock-divisor(s)
  signal clk2div_en : std_logic; -- enable for 387.5 / 2 = 193.95
  signal clk8div_en : std_logic; -- enable for 387.5 / 8 = 48.4375

  signal locked_int : std_logic;

  signal dbg_intA : std_logic;
  signal dbg_intB : std_logic;

  signal dbg_ff1 : std_logic;
  signal dbg_ff2 : std_logic;




  -- what is switch polarity?
  signal sw_meta1 : std_logic_vector(15 downto 0);
  signal sw_meta0 : std_logic_vector(15 downto 0);
  -- debounce counter
  signal bounce_counter : unsigned(7 downto 0) := "00000000";


  signal sw_sample : std_logic_vector(15 downto 0) := "0000000000000000";
  
-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  component machine is
  Port (

--         pixelclock : STD_LOGIC;
--         pixelclock2x : STD_LOGIC;
--         cpuclock : std_logic;
--         clock50mhz : std_logic;
--         ioclock : std_logic;
--         uartclock : std_logic;
--         btnCpuReset : in  STD_LOGIC;

    sysclk : in std_logic;
    reset2 : in std_logic;
    pixelclock_en : in std_logic;
    cpuioclock_en : in std_logic;

-- machine
         irq : in  STD_LOGIC;
         nmi : in  STD_LOGIC;

--         no_kickstart : in std_logic;
--
--         ddr_counter : in unsigned(7 downto 0);
--         ddr_state : in unsigned(7 downto 0);
         
-- machine
         ----------------------------------------------------------------------
         -- VGA output
         ----------------------------------------------------------------------
         vsync : out  STD_LOGIC;
         hsync : out  STD_LOGIC;
         vgared : out  UNSIGNED (3 downto 0);
         vgagreen : out  UNSIGNED (3 downto 0);
         vgablue : out  UNSIGNED (3 downto 0);

--         ---------------------------------------------------------------------------
--         -- IO lines to the ethernet controller
--         ---------------------------------------------------------------------------
--         eth_mdio : inout std_logic;
--         eth_mdc : out std_logic;
--         eth_reset : out std_logic;
--         eth_rxd : in unsigned(1 downto 0);
--         eth_txd : out unsigned(1 downto 0);
--         eth_txen : out std_logic;
--         eth_rxer : in std_logic;
--         eth_rxdv : in std_logic;
--         eth_interrupt : in std_logic;
--
-- BG removed because does not provide core functionality ethernet/framepacker
         
         -------------------------------------------------------------------------
         -- Lines for the SDcard interface itself
         -------------------------------------------------------------------------
-- machine
--         cs_bo : out std_logic;
--         sclk_o : out std_logic;
--         mosi_o : out std_logic;
--         miso_i : in  std_logic;

         ---------------------------------------------------------------------------
         -- Lines for other devices that we handle here
         ---------------------------------------------------------------------------
--         aclMISO : in std_logic;
--         aclMOSI : out std_logic;
--         aclSS : out std_logic;
--         aclSCK : out std_logic;
--         aclInt1 : in std_logic;
--         aclInt2 : in std_logic;
    
-- machine
--         micData : in std_logic;
--         micClk : out std_logic;
--         micLRSel : out std_logic;
--
--         ampPWM : out std_logic;
--         ampSD : out std_logic;
--
--         tmpSDA : out std_logic;
--         tmpSCL : out std_logic;
--         tmpInt : in std_logic;
--         tmpCT : in std_logic;

         ----------------------------------------------------------------------
         -- Flash RAM for holding config
         ----------------------------------------------------------------------
-- machine
--         QspiSCK : out std_logic;
--         QspiDB : inout std_logic_vector(3 downto 0);
--         QspiCSn : out std_logic;
--
-- BG removed as ???

--         -- Temperature of FPGA
--         fpga_temperature : in std_logic_vector(11 downto 0);
         
         ---------------------------------------------------------------------------
         -- Interface to Slow RAM (wrapper around a 128MB DDR2 RAM chip)
         ---------------------------------------------------------------------------
-- machine
--         slowram_addr : out std_logic_vector(26 downto 0);
--         slowram_we : out std_logic;
--         slowram_request_toggle : out std_logic;
--         slowram_done_toggle : in std_logic;
--         slowram_datain : out std_logic_vector(7 downto 0);
--         slowram_addr_reflect : in std_logic_vector(26 downto 0);
--         slowram_datain_reflect : in std_logic_vector(7 downto 0);
--
--         -- simple-dual-port cache RAM interface so that CPU doesn't have to read
--         -- data cross-clock
--         cache_address        : out std_logic_vector(8 downto 0);
--         cache_read_data      : in std_logic_vector(150 downto 0);   
         
         ----------------------------------------------------------------------
         -- PS/2 adapted USB keyboard & joystick connector.
         -- For now we will use a keyrah adapter to connect to the keyboard.
         ----------------------------------------------------------------------
-- machine
--         ps2data : in std_logic;
--         ps2clock : in std_logic;         

         ----------------------------------------------------------------------
         -- PMOD interface for keyboard, joystick, expansion port etc board.
         ----------------------------------------------------------------------
--         pmod_clock : in std_logic;
--         pmod_start_of_sequence : in std_logic;
--         pmod_data_in : in std_logic_vector(3 downto 0);
--         pmod_data_out : out std_logic_vector(1 downto 0);
--			
--         pmoda : inout std_logic_vector(7 downto 0);
--         uart_rx : in std_logic;
--         uart_tx : out std_logic;
--
-- BG removed this as it seems not required for core functionality

         ----------------------------------------------------------------------
         -- Debug interfaces on Nexys4 board
         ----------------------------------------------------------------------
         led : out std_logic_vector(15 downto 0);
         sw : in std_logic_vector(15 downto 0);
	 
--         btn : in std_logic_vector(4 downto 0);
--
--         UART_TXD : out std_logic;
--         RsRx : in std_logic;
         
         sseg_ca : out std_logic_vector(7 downto 0);
         sseg_an : out std_logic_vector(7 downto 0)
         );
  end component; -- machine

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####
-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

begin

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

  -- input global clock buffer on the external 100M osc
  clkin_buf : IBUFG
  port map
   (I   => clk_in,
    O   => clk100buf );

  -- insert our clock/reset handling component
  clkgen0: component clkgen
  port map (
    clk_in   => clk100buf,
    reset_ext_in => btnCpuReset_in,
    clk0_out => open,
    clk1_out => clock387mhz,
    locked_out => locked_int, -- see below for description
    dbg_rstA => dbg_intA,
    dbg_rstB => dbg_intB
  );
  
  -- generate a counter that can be used to derive the slower clocks
  process(clock387mhz, locked_int) is
  begin
    if locked_int = '0' then -- reset clause, wait for MMCM to lock
      clkdiv <= "000";
		elsif rising_edge(clock387mhz) then
      clkdiv <= clkdiv + "001";
    end if;
  end process;

  -- 0th-bit of clkdiv is clk/2
  clk2div_en <= '1';--when (clkdiv(0) = '1') else '0';
  -- 3-bit counter overflows every 8 clk's, or clk/8
  clk8div_en <= '1' when (clkdiv = "111" ) else '0';

  -- DEBUG: generate a TFF from clk2div_en
  process(clock387mhz, locked_int) is
  begin
    if locked_int = '0' then -- reset clause
      dbg_ff1 <= '0';
    elsif rising_edge(clock387mhz) then
      if (clk2div_en = '1') then
        dbg_ff1 <= not dbg_ff1;
      end if;
    end if;
  end process;

  -- DEBUG: generate a TFF from clk8div_en
  process(clock387mhz, locked_int) is
  begin
    if locked_int = '0' then -- reset clause
      dbg_ff2 <= '0';
    elsif rising_edge(clock387mhz) then
      if (clk8div_en = '1') then
        dbg_ff2 <= not dbg_ff2;
      end if;
    end if;
  end process;

  -- generate a delayed version of the reset signal
  --
  -- NOTE: locked_int is initially '0', and when the clkgen-component
  --       has stabilised its clock-outputs, then locked_int = '1'.
  -- NOTE: if the external reset button is asserted, the clkgen-component
  --       will deassert locked_int and locked_int will become '0'.
  -- NOTE: after the external reset button is deasserted, there is a delay
  --       while the clkgen-component acquires 'lock' onto the input-clock,
  --       during this delay, locked_int is deasserted, ie: '0', until
  --       'lock' is acquired, and then locked_int becomes asserted at '1'.
  -- NOTE: so the clock-outputs of the clkgen-component should not be used
  --       until the locked_int becomes asserted.
  --
  -- reset_delayed will be active low (to be consistent with existing design)
  -- so, initially reset_delayed(0) will be low, and will become un-asserted
  -- 8x clk-cycles after locked signal is acquired.
  -- this 8x clock-cycle delay can be used to route the external inputs
  -- into the design before the reset_delayed(0) is de-asserted.
  --
  -- Actually, instead of just 8x clock cycles, lets make it 8x 256x clock cycles
  -- to ensure that the external inputs have stopped bouncing (overkill but easy).
  --
  -- so, reset_delayed(0) = 1 -> circuit runs as normal
  -- and reset_delayed(0) = 0 -> circuit becomes reset
  --
  -- counter to minimise debouncing effects on the inputs
  -- basically a up-counter, forever loops
  process(clock387mhz, locked_int) is
  begin
    if (locked_int = '0') then
      -- system is in reset until locked_int becomes high
      bounce_counter <= (others => '0');
    elsif rising_edge(clock387mhz) then
      bounce_counter <= bounce_counter + 1;
    end if;
  end process;
  --
  process (clock387mhz, locked_int) is
  begin
    if rising_edge(clock387mhz) then
      if bounce_counter = "11111111" then
        reset_delayed <= locked_int & reset_delayed(7 downto 1);
      end if;
    end if;
  end process;
  
  -- combine individual switch inputs to a common bus (to ease coding)
  -- NOTE that we cannot declare (at the top-level) the 16x switches as a common-bus,
  --      because the iostandards are different for some switches,
  --      and because of this difference, ISE complains with a warning that
  --      different iostandards are used on a bus.
  sw_int <= sw_inF & sw_inE & sw_inD & sw_inC &
            sw_inB & sw_inA & sw_in9 & sw_in8 &
            sw_in7 & sw_in6 & sw_in5 & sw_in4 &
            sw_in3 & sw_in2 & sw_in1 & sw_in0;

  -- metastability for the external inputs
  process (clock387mhz, locked_int) is
  begin
    if (locked_int = '0') then
      -- system is in reset until locked_int becomes high
      sw_meta1 <= (others => '0');
      sw_meta0 <= (others => '0');
    elsif rising_edge(clock387mhz) then
      sw_meta1 <= sw_int; -- the "SW"itch bus (15..0)
      sw_meta0 <= sw_meta1;
    end if;
  end process;

  -- sample the metastable inputs whenever the bounce_counter is '1'
  process(clock387mhz, locked_int) is
  begin
    if (locked_int = '0') then
      -- system is in reset until locked_int becomes high
      sw_sample <= (others => '0');
    elsif rising_edge(clock387mhz) then
      if bounce_counter = "11111111" then
        -- sample bouncing switch input signals (POST METASTABLE)
        sw_sample <= sw_meta0;
      else
        sw_sample <= sw_sample; -- else hold sample steady
      end if;
    end if;
  end process;

  ja1_out <= dbg_intA;
  ja2_out <= dbg_intB;
  ja3_out <= btnCpuReset_in;
  ja4_out <= locked_int;
  
  ja7_out <= '1';--clock100mhz;
  ja8_out <= reset_delayed(0);
  ja9_out <= dbg_ff1;
  jaa_out <= dbg_ff2;
  
-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####
  
  machine0: machine
    port map (
      sysclk => clock387mhz,
      reset2 => reset_delayed(0), -- normally high, reset=0

      pixelclock_en => clk2div_en,
      cpuioclock_en => clk8div_en,
		
--      pixelclock2x    => open,
--      pixelclock      => open,
--      cpuclock        => open,
----      clock50mhz      => clock50mhz,
----      ioclock         => ioclock, -- 32MHz
----      uartclock       => ioclock, -- must be 32MHz
----      uartclock       => cpuclock, -- Match CPU clock (48MHz)
--      ioclock         => open, -- Match CPU clock
--		
--      btncpureset => locked_int, -- locked is active high
		
      irq => sw_meta0(0),
      nmi => sw_meta0(1),

--      no_kickstart => '0',
--      ddr_counter => ddr_counter,
--      ddr_state => ddr_state,
      
      vsync           => vsync,
      hsync           => hsync,
      vgared          => vgared,
      vgagreen        => vgagreen,
      vgablue         => vgablue,

      ---------------------------------------------------------------------------
      -- IO lines to the ethernet controller
      ---------------------------------------------------------------------------
--      eth_mdio => eth_mdio,
--      eth_mdc => eth_mdc,
--      eth_reset => eth_reset,
--      eth_rxd => eth_rxd,
--      eth_txd => eth_txd,
--      eth_txen => eth_txen,
--      eth_rxer => eth_rxer,
--      eth_rxdv => eth_rxdv,
--      eth_interrupt => eth_interrupt,
--
-- BG removed because does not provide core functionality
      
      -------------------------------------------------------------------------
      -- Lines for the SDcard interface itself
      -------------------------------------------------------------------------
-- machine
--      cs_bo => sdReset,
--      sclk_o => sdClock,
--      mosi_o => sdMOSI,
--      miso_i => sdMISO,
--
--      aclMISO => aclMISO,
--      aclMOSI => aclMOSI,
--      aclSS => aclSS,
--      aclSCK => aclSCK,
--      aclInt1 => aclInt1,
--      aclInt2 => aclInt2,
--    
--      micData => micData,
--      micClk => micClk,
--      micLRSel => micLRSel,
--
-- machine
--      ampPWM => ampPWM,
--      ampSD => ampSD,
--    
--      tmpSDA => tmpSDA,
--      tmpSCL => tmpSCL,
--      tmpInt => tmpInt,
--      tmpCT => tmpCT,
--      
--      ps2data =>      ps2data,
--      ps2clock =>     ps2clk,
--
--      pmod_clock => jblo(1),
--      pmod_start_of_sequence => jblo(2),
--      pmod_data_in(1 downto 0) => jblo(4 downto 3),
--      pmod_data_in(3 downto 2) => jbhi(8 downto 7),
--      pmod_data_out => jbhi(10 downto 9),
--      pmoda(3 downto 0) => jalo(4 downto 1),
--      pmoda(7 downto 4) => jahi(10 downto 7),
--
--      uart_rx => jclo(1),
--      uart_tx => jclo(2),
--
-- BG removed as it seems not required for core functionality

      
-- machine
--      slowram_we => slowram_we,
--      slowram_request_toggle => slowram_request_toggle,
--      slowram_done_toggle => slowram_done_toggle,
--      slowram_datain => slowram_datain,
--      slowram_addr => slowram_addr,
--      slowram_addr_reflect => slowram_addr_reflect,
--      slowram_datain_reflect => slowram_datain_reflect,
--      cache_read_data => cache_read_data,
--      cache_address => cache_address,

----      QspiSCK => QspiSCK,
--      QspiDB => QspiDB,
--      QspiCSn => QspiCSn,
--
-- BG removed as ???
--      fpga_temperature => fpga_temperature,
-- machine
      led => led_out,
      sw => sw_sample,
--      btn => btn,
--
--      UART_TXD => UART_TXD,
--      RsRx => RsRx,
--         
      sseg_ca => sseg_ca,
      sseg_an => sseg_an
      );
  

end Behavioral;
