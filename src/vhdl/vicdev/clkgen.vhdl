
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

  -- debounce registers
  signal bounce_counter : unsigned(7 downto 0) := (others => '0');
  signal reset_sample : std_logic_vector(7 downto 0) := (others => '1');-- initially pulled-up
  signal reset_short_press : std_logic := '1'; -- initially pulled-up
  signal reset_long_press  : std_logic := '1'; -- initially pulled-up

  -- MMCM signals
  signal clkfbout_int     : std_logic;
  signal clkout0_intA     : std_logic;
  signal clkout0_intB     : std_logic;
  signal clkout1_intA     : std_logic;
  signal clkout1_intB     : std_logic;
  -- unused MMCM signals
--  signal clkout0b_unused  : std_logic;
--  signal clkout1b_unused  : std_logic;
--  signal clkout2_unused   : std_logic;
--  signal clkout2b_unused  : std_logic;
--  signal clkout3_unused   : std_logic;
--  signal clkout3b_unused  : std_logic;
--  signal clkout4_unused   : std_logic;
--  signal clkout5_unused   : std_logic;
--  signal clkout6_unused   : std_logic;
  
  signal locked_int   : std_logic;
  signal locked_meta1 : std_logic;
  signal locked_meta0 : std_logic;

begin

  -- metastability logic for the external reset input
  -- continously samples without a reset
  --
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
      resetextin_meta1 <= reset_ext_in; -- the "CPU_Reset"-button
      resetextin_meta0 <= resetextin_meta1;
    end if;
  end process;
  
  -- debounce logic for the meta-stable reset signal:
  --
  -- an up-counter, forever loops, never resets
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
      bounce_counter <= bounce_counter + 1;
    end if;
  end process;
  -- sample the bouncing input whenever the bounce_counter is '1'
  -- implements a shift-register, so no reset logic
  process(clk_in) is
  begin
    if rising_edge(clk_in) then
      if bounce_counter = (bounce_counter'HIGH) then
        -- sample bouncing reset input signal (POST METASTABLE)
--        reset_sample(15) <= resetextin_meta0; -- normally high, active low
--        reset_sample(14) <= reset_sample(15);
--        reset_sample(13) <= reset_sample(14);
--        reset_sample(12) <= reset_sample(13);
--        reset_sample(11) <= reset_sample(12);
--        reset_sample(10) <= reset_sample(11);
--        reset_sample(9) <= reset_sample(10);
--        reset_sample(8) <= reset_sample(9);
--        reset_sample(7) <= reset_sample(8);
        reset_sample(7) <= resetextin_meta0; -- normally high, active low
        reset_sample(6) <= reset_sample(7);
        reset_sample(5) <= reset_sample(6);
        reset_sample(4) <= reset_sample(5);
        reset_sample(3) <= reset_sample(4);
        reset_sample(2) <= reset_sample(3);
        reset_sample(1) <= reset_sample(2);
        reset_sample(0) <= reset_sample(1);
      end if;
    end if;
  end process;
  --
  -- two 'reset' signals are implemented:
  --  * a short-press implements a warm-reset, and
  --  * a long--press implements a major-reset.
  --
  -- lets make active-high reset signals (opposed to the active-low CPU_Reset button input)
  -- we will generate a signal which is normally LOW,
  -- but becomes HIGH after the CPU_Reset button is pressed  for some period of time,
  -- and becomes LOW  after the CPU_Reset button is released for some period of time.
  process(clk_in, reset_sample, reset_short_press) is
  begin
    if rising_edge(clk_in) then
      if reset_short_press = '0' then        -- normally LOW
        if reset_sample(7 downto 4) = "0000" then   -- button pressed, signal stable
          reset_short_press <= '1';          -- internal circuit becomes active-high-RESET
        else
          reset_short_press <= reset_short_press;
        end if;
      else
      -- reset_short_press = '1'             -- internal circuit is in active-high-RESET
        if reset_sample(7 downto 4) = "1111" then   -- wait for button released, signal stable
          reset_short_press <= '0';          -- then synchronously release the active-high reset-signal
        else
          reset_short_press <= reset_short_press;
        end if;
      end if;
    end if;
  end process;
  -- similar to the above
  process(clk_in, reset_sample, reset_long_press) is
  begin
    if rising_edge(clk_in) then
      if reset_long_press = '0' then        -- normally LOW
        if reset_sample = "00000000" then   -- button pressed, signal stable
          reset_long_press <= '1';          -- internal circuit becomes active-high-RESET
        else
          reset_long_press <= reset_long_press;
        end if;
      else
      -- reset_long_press = '1'             -- internal circuit is in active-high-RESET
        if reset_sample = "11111111" then   -- wait for button released, signal stable
          reset_long_press <= '0';          -- then synchronously release the active-high reset-signal
        else
          reset_long_press <= reset_long_press;
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
      CLKFBOUT            => clkfbout_int,
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
      CLKFBIN             => clkfbout_int,
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
      RST                 => reset_long_press -- MMCM uses active-high reset signal
    );
     

  clkout0_buf : BUFG
  port map (
    I   => clkout0_intA,
    O   => clkout0_intB
    );
  
  clkout1_buf : BUFG
  port map (
    I   => clkout1_intA,
    O   => clkout1_intB
    );

  -- component outputs
  clk0_out <= clkout0_intB;
  clk1_out <= clkout1_intB;

  locked_out <= locked_int;
  
  dbg_rstA <= reset_short_press;
  dbg_rstB <= reset_long_press;
  
end Behavioral;
