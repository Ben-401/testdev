----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
--use ieee.std_logic_arith.all;
--use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use Std.TextIO.all;
use work.debugtools.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

-- initially modelled from:
-- WEBSITE: https://github.com/jakubcabal/uart_for_fpga

entity uart is
  generic (
    CLK_FREQ   : integer := 148e06;  -- set system clock frequency in Hz
    BAUD_RATE  : integer := 230400;    -- baud rate value
    PARITY_BIT : string  := "none"     -- legal values: "none", "even", "odd", "mark", "space"
    );
  port (
    sysclk        : in std_logic;
	  reset_S       : in std_logic;
	  cpuioclock_en : in std_logic;
	  txd           : out std_logic;
	  rxd           : in  std_logic
    );
end uart;

-- ####### ####### ####### ####### ####### ####### ####### ####### ####### ####

architecture Behavioral of uart is

  constant divider_value    : integer := CLK_FREQ/(16*BAUD_RATE);

  signal uart_ticks         : integer range 0 to divider_value-1;
  signal uart_clk_en        : std_logic;
  signal uart_rxd_shreg     : std_logic_vector(3 downto 0);
  signal uart_rxd_debounced : std_logic;
    
  signal data    : std_logic_vector(7 downto 0);
  signal valid   : std_logic;

begin

  -- -------------------------------------------------------------------------
  -- UART OVERSAMPLING CLOCK DIVIDER
  -- -------------------------------------------------------------------------

  uart_oversampling_clk_divider : process (sysclk, cpuioclock_en)
  begin
    if (rising_edge(sysclk) and cpuioclock_en='1') then
      if (reset_S = '1') then
        uart_ticks <= 0;
        uart_clk_en <= '0';
      elsif (uart_ticks = divider_value-1) then
        uart_ticks <= 0;
        uart_clk_en <= '1';
      else
        uart_ticks <= uart_ticks + 1;
        uart_clk_en <= '0';
      end if;
    end if;
  end process;

  -- -------------------------------------------------------------------------
  -- UART RXD DEBAUNCER
  -- -------------------------------------------------------------------------

  uart_rxd_debouncer : process (sysclk, cpuioclock_en)
  begin
    if (rising_edge(sysclk) and cpuioclock_en='1') then
      if (reset_S = '1') then
        uart_rxd_shreg <= (others => '1');
        uart_rxd_debounced <= '1';
      else
        uart_rxd_shreg <= rxd & uart_rxd_shreg(3 downto 1);
        uart_rxd_debounced <= uart_rxd_shreg(0) OR
                              uart_rxd_shreg(1) OR
                              uart_rxd_shreg(2) OR
                              uart_rxd_shreg(3);
      end if;
    end if;
  end process;

  -- -------------------------------------------------------------------------
  -- UART TRANSMITTER
  -- -------------------------------------------------------------------------

  uart_tx_i: entity work.UART_TX
  generic map (
    PARITY_BIT => PARITY_BIT
    )
  port map (
    CLK         => sysclk,
    RST         => reset_S,
    -- UART INTERFACE
    UART_CLK_EN => uart_clk_en,
    UART_TXD    => txd,
    -- USER DATA INPUT INTERFACE
    DATA_IN     => data, --DATA_IN,
    DATA_SEND   => valid, --DATA_SEND,
    BUSY        => open --BUSY
    );

  -- -------------------------------------------------------------------------
  -- UART RECEIVER
  -- -------------------------------------------------------------------------

  uart_rx_i: entity work.UART_RX
  generic map (
    PARITY_BIT => PARITY_BIT
    )
  port map (
    CLK         => sysclk,
    RST         => reset_S,
    -- UART INTERFACE
    UART_CLK_EN => uart_clk_en,
    UART_RXD    => uart_rxd_debounced,
    -- USER DATA OUTPUT INTERFACE
    DATA_OUT    => data, --DATA_OUT,
    DATA_VLD    => valid, --DATA_VLD,
    FRAME_ERROR => open --FRAME_ERROR
    );
    
end Behavioral;

