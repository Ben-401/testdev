
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;

library UNISIM;
use UNISIM.VComponents.all;

entity clkgen is
  Port (
    clk_in : STD_LOGIC;           -- from/through the IBUFG
    reset_ext_in : in  STD_LOGIC; -- direct from external, is normally high, active low
         
    clk0_out : out std_logic; -- want this 100mhz
    clk1_out : out std_logic; -- want this about 193.5mhz
	 
    locked_out : out std_logic;
	 
	 dbg_rstA : out std_logic;
	 dbg_rstB : out std_logic
	 
  );
end clkgen;

architecture Behavioral of clkgen is

  -- metastability for inputs
  
  -- The external "CPU_Reset"-input has PULL-UP, so:
  -- normally-open  = "1"
  -- button-pressed = "0" = active-low reset-signal  
  signal resetextin_meta1 : std_logic := '1'; -- initially pull-up
  signal resetextin_meta0 : std_logic := '1';

  -- debounce counter
  signal bounce_counter : unsigned(7 downto 0) := "00000000";

  signal reset_sample : std_logic_vector(3 downto 0) := "1111"; -- initially pulled-up
  signal reset_stable : std_logic := '1'; -- initially pulled-up

  signal clkfbout_intA     : std_logic;
--  signal clkfbout_intB     : std_logic;
  signal clkout0_intA     : std_logic;
  signal clkout0_intB     : std_logic;
  signal clkout1_intA     : std_logic;
  signal clkout1_intB     : std_logic;

  signal clkfboutb_unused : std_logic;
  signal clkout0b_unused  : std_logic;
  signal clkout1b_unused  : std_logic;
  signal clkout2_unused   : std_logic;
  signal clkout2b_unused  : std_logic;
  signal clkout3_unused   : std_logic;
  signal clkout3b_unused  : std_logic;
  signal clkout4_unused   : std_logic;
  signal clkout5_unused   : std_logic;
  signal clkout6_unused   : std_logic;
  
  signal locked_int   : std_logic;
  signal locked_meta1 : std_logic;
  signal locked_meta0 : std_logic;

begin

  -- metastability for the external reset input
  -- continously samples without a reset
  --
  process (clk_in) is
  begin
    if rising_edge(clk_in) then
      resetextin_meta1 <= reset_ext_in; -- the "CPU_Reset"-button
      resetextin_meta0 <= resetextin_meta1;
    end if;
  end process;
  
  -- counter to minimise debouncing effects on the reset input
  -- basically an up-counter, forever loops
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
      bounce_counter <= bounce_counter + 1;
    end if;
  end process;
  -- sample the bouncing input whenever the bounce_counter is '1'
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
	   if bounce_counter = "11111111" then
		  -- sample bouncing reset input signal (POST METASTABLE)
		  reset_sample(3) <= resetextin_meta0; -- normally high, active low
		  reset_sample(2) <= reset_sample(3);
		  reset_sample(1) <= reset_sample(2);
		  reset_sample(0) <= reset_sample(1);
		end if;
    end if;
  end process;
  
  -- lets make an active-high reset signal (opposed to the active-low CPU_Reset input)
  -- we will generate a signal which is normally LOW,
  -- but becomes HIGH after the CPU_Reset button is pressed  for some period of time,
  -- and becomes LOW  after the CPU_Reset button is released for some period of time.
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
	   if reset_stable = '0' then        -- normally LOW
		  if reset_sample = "0000" then   -- button pressed, signal stable
		    reset_stable <= '1';          -- internal circuit becomes active-high-RESET
		  else
		    reset_stable <= reset_stable;
		  end if;
		else
		-- reset_stab = '1'             -- internal circuit is in active-high-RESET
		  if reset_sample = "1111" then   -- wait for button released, signal stable
		    reset_stable <= '0';          -- then synchronously release the active-high reset-signal
		  else
		    reset_stable <= reset_stable;
		  end if;
		end if;
    end if;
  end process;

  -- insert a MMCM to:
  -- - create a stable output clock at 100mhz
  -- - create a stable output clock at 193.18387.5mhz
  mmcm_adv_inst: MMCME2_ADV
    generic map(
	   BANDWIDTH            => "OPTIMIZED",
		CLKOUT4_CASCADE      => FALSE,
		COMPENSATION         => "INTERNAL",
      STARTUP_WAIT         => FALSE,
		DIVCLK_DIVIDE        => 1,
		CLKFBOUT_MULT_F      => 10.375,--11.625, -- is input multiplier
		CLKFBOUT_PHASE       => 0.0,
      CLKFBOUT_USE_FINE_PS => FALSE,
		
      CLKOUT0_DIVIDE_F     => 10.375,--11.625, -- is output divisor
      CLKOUT0_PHASE        => 0.0,
      CLKOUT0_DUTY_CYCLE   => 0.5,
      CLKOUT0_USE_FINE_PS  => FALSE,
			 
      CLKOUT1_DIVIDE       => 7,--3,    -- is output divisor
      CLKOUT1_PHASE        => 0.0,
      CLKOUT1_DUTY_CYCLE   => 0.5,
      CLKOUT1_USE_FINE_PS  => FALSE,
			 
      CLKIN1_PERIOD        => 10.000,
      REF_JITTER1          => 0.100
    )
    port map(
	   -- Output clocks
		CLKFBOUT            => clkfbout_intA,
		CLKFBOUTB           => open,
		CLKOUT0             => clkout0_intA,
		CLKOUT0B            => open,
		CLKOUT1             => clkout1_intA,
		CLKOUT1B            => open,
		CLKOUT2             => open,
      CLKOUT2B            => open,
      CLKOUT3             => open,
      CLKOUT3B            => open,
      CLKOUT4             => open,
      CLKOUT5             => open,
      CLKOUT6             => open,
      -- Input clock control
      CLKFBIN             => clkfbout_intA,
      CLKIN1              => clk_in,
      CLKIN2              => clk_in,
      -- Tied to always select the primary input clock
      CLKINSEL            => '1',
      -- Ports for dynamic reconfiguration
      DADDR               => (others => '0'),
      DCLK                => '0',
      DEN                 => '0',
      DI                  => (others => '0'),
      DO                  => open,
      DRDY                => open,
      DWE                 => '0',
      -- Ports for dynamic phase shift
      PSCLK               => '0',
      PSEN                => '0',
      PSINCDEC            => '0',
      PSDONE              => open,
      -- Other control and status signals
      LOCKED              => locked_int, -- MMCM generates active-high locked signal
      CLKINSTOPPED        => open,
      CLKFBSTOPPED        => open,
      PWRDWN              => '0',
      RST                 => reset_stable -- MMCM uses active-high reset signal
    );
     
--  clkfb_buf : BUFG
--  port map (
--    I   => clkfbout_intA,
--	 O   => clkfbout_intB );
	 
  clkout0_buf : BUFG
  port map (
    I   => clkout0_intA,
	 O   => clkout0_intB );
  
  clkout1_buf : BUFG
  port map (
    I   => clkout1_intA,
	 O   => clkout1_intB );

  -- generate a locked-signal that is synchronised with the clk387
  process(clkout1_intB, locked_int) is
  begin
    if rising_edge(clkout1_intB) then
      locked_meta1 <= locked_int;
      locked_meta0 <= locked_meta1;
    end if;
  end process;
  
  
  
  -- component outputs
  clk0_out <= clkout0_intB;
  clk1_out <= clkout1_intB;

  locked_out <= locked_meta0;
  
  dbg_rstA <= resetextin_meta0;
  dbg_rstB <= reset_stable;
  
end Behavioral;
