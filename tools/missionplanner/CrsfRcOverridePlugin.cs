// CRSF RC Override — плагін Mission Planner.
//
// Читає кадри CRSF RC_CHANNELS_PACKED (0x16) з COM-порту (наприклад, EdgeTX
// у режимі "USB RC OUT" або CRSF/ELRS-приймач через USB-UART) і пересилає
// канали на підключений борт повідомленнями MAVLink RC_CHANNELS_OVERRIDE.
//
// Встановлення: скопіювати цей файл у папку "plugins" поруч із
// MissionPlanner.exe і перезапустити Mission Planner.
// Меню: Flight Data → правий клік по карті → "CRSF RC Override...".
//
// Безпека:
//  - якщо кадри CRSF перестали надходити (> FrameTimeoutMs), плагін
//    припиняє override і повертає канали RC-приймачу борту;
//  - на борту: MAV_GCS_SYSID має дорівнювати SYSID Mission Planner (255),
//    RC_OVERRIDE_TIME задає, скільки борт тримає останній override.
//  - не вмикайте одночасно штатний Joystick Mission Planner.

using System;
using System.Drawing;
using System.IO.Ports;
using System.Threading;
using System.Windows.Forms;
using MissionPlanner;
using MissionPlanner.Plugin;
using MissionPlanner.Utilities;

namespace CrsfRcOverride
{
    public class CrsfRcOverridePlugin : Plugin
    {
        public override string Name => "CRSF RC Override";
        public override string Version => "0.1";
        public override string Author => "leshcom";

        internal const int MaxChannels = 16;
        internal const int FrameTimeoutMs = 200;

        internal static CrsfRcOverridePlugin Instance;

        // --- стан, спільний між потоками ---
        readonly object _lock = new object();
        readonly ushort[] _pwm = new ushort[MaxChannels];
        DateTime _lastFrameUtc = DateTime.MinValue;

        SerialPort _port;
        Thread _rxThread;
        Thread _txThread;
        volatile bool _rxRun;
        volatile bool _txRun;

        // налаштування (змінюються з UI)
        internal volatile bool OverrideEnabled;
        internal volatile int OverrideChannels = 16;
        internal volatile int SendRateHz = 50;

        // статистика
        internal long FramesOk;
        internal long CrcErrors;
        internal long PacketsSent;
        internal volatile string LastError = "";
        internal volatile bool OverrideActive;

        CrsfForm _form;

        public override bool Init()
        {
            Instance = this;
            return true;
        }

        public override bool Loaded()
        {
            var item = new ToolStripMenuItem("CRSF RC Override...");
            item.Click += (s, e) => ShowForm();
            Host.FDMenuMap.Items.Add(item);

            _txRun = true;
            _txThread = new Thread(TxLoop) { IsBackground = true, Name = "CRSF RC override TX" };
            _txThread.Start();
            return true;
        }

        public override bool Exit()
        {
            // TX-потік сам надішле release, побачивши OverrideEnabled = false
            OverrideEnabled = false;
            Thread.Sleep(150 + 1000 / Math.Max(1, SendRateHz));
            _txRun = false;
            ClosePort();
            try { _txThread?.Join(500); } catch { }
            return true;
        }

        void ShowForm()
        {
            if (_form == null || _form.IsDisposed)
                _form = new CrsfForm(this);
            _form.Show();
            _form.BringToFront();
        }

        // ------------------------------------------------------------------
        // COM-порт і розбір CRSF
        // ------------------------------------------------------------------

        internal bool PortOpen => _port != null && _port.IsOpen;

        internal void OpenPort(string name, int baud)
        {
            ClosePort();
            try
            {
                _port = new SerialPort(name, baud, Parity.None, 8, StopBits.One)
                {
                    ReadTimeout = 100,
                    DtrEnable = true,
                    RtsEnable = true,
                };
                _port.Open();
                _rxRun = true;
                _rxThread = new Thread(RxLoop) { IsBackground = true, Name = "CRSF RC override RX" };
                _rxThread.Start();
                LastError = "";
                Settings.Instance["crsf_rc_port"] = name;
                Settings.Instance["crsf_rc_baud"] = baud.ToString();
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
                ClosePort();
            }
        }

        internal void ClosePort()
        {
            _rxRun = false;
            var p = _port;
            _port = null;
            try { p?.Close(); } catch { }
            try { _rxThread?.Join(500); } catch { }
            _rxThread = null;
        }

        void RxLoop()
        {
            var parser = new CrsfParser();
            parser.OnChannels = OnChannels;
            parser.OnCrcError = () => Interlocked.Increment(ref CrcErrors);
            var buf = new byte[256];

            while (_rxRun)
            {
                var p = _port;
                if (p == null) break;
                try
                {
                    int n = p.Read(buf, 0, buf.Length);
                    if (n > 0) parser.Feed(buf, n);
                }
                catch (TimeoutException) { }
                catch (Exception ex)
                {
                    // кабель витягли, режим USB змінили тощо
                    LastError = ex.Message;
                    _rxRun = false;
                    try { p.Close(); } catch { }
                }
            }
        }

        void OnChannels(ushort[] crsf)
        {
            lock (_lock)
            {
                for (int i = 0; i < MaxChannels; i++)
                    _pwm[i] = CrsfParser.CrsfToUs(crsf[i]);
                _lastFrameUtc = DateTime.UtcNow;
            }
            Interlocked.Increment(ref FramesOk);
        }

        internal ushort[] GetChannels(out double ageMs)
        {
            lock (_lock)
            {
                ageMs = (DateTime.UtcNow - _lastFrameUtc).TotalMilliseconds;
                return (ushort[])_pwm.Clone();
            }
        }

        // ------------------------------------------------------------------
        // Відправка RC_CHANNELS_OVERRIDE
        // ------------------------------------------------------------------

        void TxLoop()
        {
            bool wasActive = false;

            while (_txRun)
            {
                int rate = Math.Max(1, Math.Min(100, SendRateHz));
                Thread.Sleep(1000 / rate);

                try
                {
                    var com = MainV2.comPort;
                    if (com?.BaseStream == null || !com.BaseStream.IsOpen)
                    {
                        wasActive = false;
                        OverrideActive = false;
                        continue;
                    }

                    var ch = GetChannels(out double age);
                    bool fresh = age < FrameTimeoutMs;
                    bool active = OverrideEnabled && fresh;

                    if (active)
                    {
                        // не забиваємо телеметрійний канал (як штатний Joystick MP)
                        if (com.BaseStream.BytesToWrite < 50)
                        {
                            var rc = BuildOverride(com, ch, OverrideChannels);
                            com.sendPacket(rc, com.MAV.sysid, com.MAV.compid);
                            Interlocked.Increment(ref PacketsSent);
                        }
                    }
                    else if (wasActive)
                    {
                        // повернути керування RC-приймачу борту
                        var rc = BuildRelease(com);
                        for (int i = 0; i < 3; i++)
                        {
                            com.sendPacket(rc, com.MAV.sysid, com.MAV.compid);
                            Thread.Sleep(20);
                        }
                    }

                    wasActive = active;
                    OverrideActive = active;
                }
                catch (Exception ex)
                {
                    LastError = ex.Message;
                }
            }
        }

        static MAVLink.mavlink_rc_channels_override_t NewOverride(MAVLinkInterface com)
        {
            return new MAVLink.mavlink_rc_channels_override_t
            {
                target_system = (byte)com.MAV.sysid,
                target_component = (byte)com.MAV.compid,
            };
        }

        // Канали 1..count — значення в мкс; решта — "ігнорувати" (UINT16_MAX).
        static MAVLink.mavlink_rc_channels_override_t BuildOverride(MAVLinkInterface com, ushort[] us, int count)
        {
            var rc = NewOverride(com);
            ushort Ch(int i) => i < count ? us[i] : ushort.MaxValue;

            rc.chan1_raw = Ch(0);
            rc.chan2_raw = Ch(1);
            rc.chan3_raw = Ch(2);
            rc.chan4_raw = Ch(3);
            rc.chan5_raw = Ch(4);
            rc.chan6_raw = Ch(5);
            rc.chan7_raw = Ch(6);
            rc.chan8_raw = Ch(7);
            rc.chan9_raw = Ch(8);
            rc.chan10_raw = Ch(9);
            rc.chan11_raw = Ch(10);
            rc.chan12_raw = Ch(11);
            rc.chan13_raw = Ch(12);
            rc.chan14_raw = Ch(13);
            rc.chan15_raw = Ch(14);
            rc.chan16_raw = Ch(15);
            rc.chan17_raw = ushort.MaxValue;
            rc.chan18_raw = ushort.MaxValue;
            return rc;
        }

        // MAVLink: ch1-8 = 0 → release; ch9-18 = UINT16_MAX-1 → release.
        static MAVLink.mavlink_rc_channels_override_t BuildRelease(MAVLinkInterface com)
        {
            var rc = NewOverride(com);
            const ushort rel = ushort.MaxValue - 1;
            rc.chan1_raw = 0; rc.chan2_raw = 0; rc.chan3_raw = 0; rc.chan4_raw = 0;
            rc.chan5_raw = 0; rc.chan6_raw = 0; rc.chan7_raw = 0; rc.chan8_raw = 0;
            rc.chan9_raw = rel; rc.chan10_raw = rel; rc.chan11_raw = rel; rc.chan12_raw = rel;
            rc.chan13_raw = rel; rc.chan14_raw = rel; rc.chan15_raw = rel; rc.chan16_raw = rel;
            rc.chan17_raw = rel; rc.chan18_raw = rel;
            return rc;
        }
    }

    // ----------------------------------------------------------------------
    // Потоковий розбір CRSF
    // Кадр: [addr][len][type][payload...][crc8]; len = type + payload + crc.
    // ----------------------------------------------------------------------
    public class CrsfParser
    {
        const int MaxFrame = 64;
        const byte TypeRcChannels = 0x16;

        readonly byte[] _buf = new byte[MaxFrame];
        int _pos;

        public Action<ushort[]> OnChannels;
        public Action OnCrcError;

        static bool IsSync(byte b) => b == 0xC8 || b == 0xEE || b == 0xEA || b == 0xEC;

        public void Feed(byte[] data, int count)
        {
            for (int i = 0; i < count; i++)
                Push(data[i]);
        }

        void Push(byte b)
        {
            if (_pos == 0)
            {
                if (IsSync(b)) _buf[_pos++] = b;
                return;
            }

            if (_pos == 1)
            {
                // допустима довжина: type + crc мінімум, весь кадр ≤ 64
                if (b < 2 || b > MaxFrame - 2)
                {
                    _pos = 0;
                    if (IsSync(b)) _buf[_pos++] = b;
                    return;
                }
                _buf[_pos++] = b;
                return;
            }

            _buf[_pos++] = b;
            int frameLen = _buf[1] + 2;
            if (_pos < frameLen) return;

            byte crc = Crc8(_buf, 2, _buf[1] - 1);
            if (crc == _buf[frameLen - 1])
            {
                if (_buf[2] == TypeRcChannels && _buf[1] == 24)
                    OnChannels?.Invoke(Unpack(_buf, 3));
                _pos = 0;
            }
            else
            {
                OnCrcError?.Invoke();
                Resync(frameLen);
            }
        }

        // Після помилки CRC шукаємо наступний sync-байт усередині буфера.
        void Resync(int len)
        {
            int start = -1;
            for (int i = 1; i < len; i++)
                if (IsSync(_buf[i])) { start = i; break; }

            _pos = 0;
            if (start < 0) return;

            var rest = new byte[len - start];
            Array.Copy(_buf, start, rest, 0, rest.Length);
            foreach (var x in rest) Push(x);
        }

        public static ushort[] Unpack(byte[] p, int offset)
        {
            var ch = new ushort[CrsfRcOverridePlugin.MaxChannels];
            uint bits = 0;
            int avail = 0;
            int idx = offset;
            for (int i = 0; i < ch.Length; i++)
            {
                while (avail < 11)
                {
                    bits |= (uint)p[idx++] << avail;
                    avail += 8;
                }
                ch[i] = (ushort)(bits & 0x7FF);
                bits >>= 11;
                avail -= 11;
            }
            return ch;
        }

        // Як в ArduPilot: 172 → 987.5, 992 → 1500, 1811 → 2011.9 мкс.
        public static ushort CrsfToUs(ushort v) => (ushort)((v * 5) / 8 + 880);

        public static byte Crc8(byte[] data, int offset, int len)
        {
            byte crc = 0;
            for (int i = 0; i < len; i++)
            {
                crc ^= data[offset + i];
                for (int b = 0; b < 8; b++)
                    crc = (crc & 0x80) != 0 ? (byte)((crc << 1) ^ 0xD5) : (byte)(crc << 1);
            }
            return crc;
        }
    }

    // ----------------------------------------------------------------------
    // Вікно налаштувань і стану
    // ----------------------------------------------------------------------
    public class CrsfForm : Form
    {
        readonly CrsfRcOverridePlugin _p;
        readonly ComboBox _cmbPort = new ComboBox { DropDownStyle = ComboBoxStyle.DropDown, Width = 110 };
        readonly ComboBox _cmbBaud = new ComboBox { DropDownStyle = ComboBoxStyle.DropDown, Width = 80 };
        readonly Button _btnRefresh = new Button { Text = "↻", Width = 28 };
        readonly Button _btnConnect = new Button { Text = "Відкрити", Width = 90 };
        readonly CheckBox _chkEnable = new CheckBox { Text = "Надсилати RC override на борт", AutoSize = true };
        readonly NumericUpDown _numCh = new NumericUpDown { Minimum = 1, Maximum = 16, Value = 16, Width = 50 };
        readonly NumericUpDown _numRate = new NumericUpDown { Minimum = 5, Maximum = 100, Value = 50, Width = 50 };
        readonly Label _lblStatus = new Label { AutoSize = true };
        readonly Label _lblChannels = new Label { AutoSize = true, Font = new Font(FontFamily.GenericMonospace, 9f) };
        readonly System.Windows.Forms.Timer _timer = new System.Windows.Forms.Timer { Interval = 200 };

        long _lastFrames;
        DateTime _lastTick = DateTime.UtcNow;

        public CrsfForm(CrsfRcOverridePlugin p)
        {
            _p = p;
            Text = "CRSF RC Override";
            FormBorderStyle = FormBorderStyle.FixedToolWindow;
            StartPosition = FormStartPosition.CenterParent;
            AutoSize = true;
            AutoSizeMode = AutoSizeMode.GrowAndShrink;
            Padding = new Padding(8);

            _cmbBaud.Items.AddRange(new object[] { "420000", "416666", "115200", "921600" });
            _cmbBaud.Text = Settings.Instance["crsf_rc_baud", "420000"];
            RefreshPorts();
            var savedPort = Settings.Instance["crsf_rc_port"];
            if (!string.IsNullOrEmpty(savedPort)) _cmbPort.Text = savedPort;

            _numCh.Value = _p.OverrideChannels;
            _numRate.Value = _p.SendRateHz;
            _chkEnable.Checked = _p.OverrideEnabled;

            var layout = new FlowLayoutPanel
            {
                FlowDirection = FlowDirection.TopDown,
                AutoSize = true,
                WrapContents = false,
                Dock = DockStyle.Fill,
            };

            var row1 = Row(new Label { Text = "Порт:", AutoSize = true, Anchor = AnchorStyles.Left },
                _cmbPort, _btnRefresh,
                new Label { Text = "Швидкість:", AutoSize = true }, _cmbBaud, _btnConnect);
            var row2 = Row(_chkEnable);
            var row3 = Row(new Label { Text = "Каналів:", AutoSize = true }, _numCh,
                new Label { Text = "Частота, Гц:", AutoSize = true }, _numRate);

            layout.Controls.Add(row1);
            layout.Controls.Add(row2);
            layout.Controls.Add(row3);
            layout.Controls.Add(_lblStatus);
            layout.Controls.Add(_lblChannels);
            Controls.Add(layout);

            _btnRefresh.Click += (s, e) => RefreshPorts();
            _btnConnect.Click += (s, e) => ToggleConnect();
            _chkEnable.CheckedChanged += (s, e) => _p.OverrideEnabled = _chkEnable.Checked;
            _numCh.ValueChanged += (s, e) => _p.OverrideChannels = (int)_numCh.Value;
            _numRate.ValueChanged += (s, e) => _p.SendRateHz = (int)_numRate.Value;
            _timer.Tick += (s, e) => UpdateStatus();
            _timer.Start();
            FormClosed += (s, e) => _timer.Stop();

            UpdateStatus();
        }

        static FlowLayoutPanel Row(params Control[] controls)
        {
            var r = new FlowLayoutPanel { AutoSize = true, WrapContents = false, Margin = new Padding(0, 2, 0, 2) };
            foreach (var c in controls)
            {
                if (c is Label) c.Margin = new Padding(3, 7, 3, 0);
                r.Controls.Add(c);
            }
            return r;
        }

        void RefreshPorts()
        {
            var cur = _cmbPort.Text;
            _cmbPort.Items.Clear();
            _cmbPort.Items.AddRange(SerialPort.GetPortNames());
            if (!string.IsNullOrEmpty(cur)) _cmbPort.Text = cur;
            else if (_cmbPort.Items.Count > 0) _cmbPort.SelectedIndex = 0;
        }

        void ToggleConnect()
        {
            if (_p.PortOpen)
            {
                _p.ClosePort();
            }
            else
            {
                if (!int.TryParse(_cmbBaud.Text, out int baud)) baud = 420000;
                _p.OpenPort(_cmbPort.Text, baud);
            }
            UpdateStatus();
        }

        void UpdateStatus()
        {
            _btnConnect.Text = _p.PortOpen ? "Закрити" : "Відкрити";

            var now = DateTime.UtcNow;
            long frames = Interlocked.Read(ref _p.FramesOk);
            double dt = (now - _lastTick).TotalSeconds;
            double fps = dt > 0 ? (frames - _lastFrames) / dt : 0;
            _lastFrames = frames;
            _lastTick = now;

            var ch = _p.GetChannels(out double age);
            bool fresh = age < CrsfRcOverridePlugin.FrameTimeoutMs;

            string link = MainV2.comPort?.BaseStream?.IsOpen == true
                ? $"борт sysid {MainV2.comPort.MAV.sysid}"
                : "борт не підключено";

            _lblStatus.Text =
                $"CRSF: {(fresh ? "є кадри" : "немає кадрів")}, {fps:0} кадр/с, " +
                $"CRC-помилок {Interlocked.Read(ref _p.CrcErrors)}\n" +
                $"{link}; override: {(_p.OverrideActive ? "АКТИВНИЙ" : "вимкнено")}, " +
                $"надіслано {Interlocked.Read(ref _p.PacketsSent)}" +
                (string.IsNullOrEmpty(_p.LastError) ? "" : $"\nПомилка: {_p.LastError}");
            _lblStatus.ForeColor = _p.OverrideActive ? Color.DarkGreen : SystemColors.ControlText;

            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < 8; i++)
                sb.AppendLine($"CH{i + 1,-2} {(fresh ? ch[i].ToString() : "----"),5}    CH{i + 9,-2} {(fresh ? ch[i + 8].ToString() : "----"),5}");
            _lblChannels.Text = sb.ToString();
        }
    }
}
