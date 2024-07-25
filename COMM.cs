using DSRemapper;
using DSRemapper.Core;
using DSRemapper.DSRMath;
using DSRemapper.SixAxis;
using DSRemapper.Types;
using FireLibs.IO.COMPorts.Win;
using SerialDeviceInfo = FireLibs.IO.COMPorts.SerialDeviceInfo;
using System.Runtime.InteropServices;
using DSRemapper.Core.Types;
using FireLibs.Logging;

namespace DSRemapper.COMM
{
    /// <summary>
    /// Structure that match COM controller info data structure
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 32)]
    internal struct COMInfoReport
    {
        public byte ReportIdCount; // If this value is more than one, DSRemapper will spect multiple Input Reports per request
        public ushort AccelerometerScale;
        public ushort GyroscopeScale;
        public BitVector<byte> Opts1 = new();
        public static BitVector<byte>.Section ReportACK = new(1,0); // Report as ACK. If set, then DSRemapper will use 0x03 Code for data retrive
        public static BitVector<byte>.Section CRC16 = new(1, 1); // if set, DSRemapper will expect a CRC16 on the last bytes of the InputReport and the OutputReport. Otherwise, this bytes can be used for data transfer
        public static BitVector<byte>.Section CustomName = new(1, 2); // if set, DSRemapper will request a custom name with 0x10 code
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 27)]
        byte[] reserved = new byte[27];

        public COMInfoReport() { }
    }

    /// <summary>
    /// Structure that match COM controller input data structure
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 64)]
    internal struct COMInputReport
    {
        public byte Id = 0; // reserved for the future (useless at the moment)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public short[] Axes = new short[6];
        public ushort Pov0 = ushort.MaxValue;
        public uint Buttons = 0;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public short[] Accelerometer = new short[3];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public short[] Gyroscope = new short[3];
        public byte Battery = 0;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public short[] Sliders = new short[6];
        public ushort Pov1 = ushort.MaxValue;
        public uint ButtonsExt = 0;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public short[] AxesExt = new short[6];
        public short CRC16=0;

        public COMInputReport() { }
    }

    // Report size is 32 bytes long, BUT this struct includes the PROTOCOL CODE as first byte, therefore it is 33 bytes long.
    // This made me waste a WHOLE DAY until I realized that the size of the structure should be 33 bytes long.
    // This comment is a reminder of my DUMBEST mistake to the date - August 10, 2023.
    /// <summary>
    /// Structure that match COM controller output data structure
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 33)]
    internal struct COMOutputReport
    {
        public readonly byte code = 2; // COM protocol code embedded into the structure
        public byte Id = 0; // reserved for the future (useless at the moment)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public short[] Axes = new short[6];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] Buttons = new byte[4];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 13)]
        byte[] reserved = new byte[13];
        public short CRC16 = 0;

        public COMOutputReport() { }
    }

    /// <summary>
    /// COM device scanner class
    /// </summary>
    public class COMMScanner : IDSRDeviceScanner
    {
        /// <summary>
        /// Gets available/connected devices as IDSInputDeviceInfo array
        /// </summary>
        /// <returns>A IDSInputDeviceInfo array containing the COM devices</returns>
        public IDSRInputDeviceInfo[] ScanDevices() => SerialPort.GetSerialDevices().Select(sd => new COMMDeviceInfo(sd)).ToArray();
    }
    /// <summary>
    /// COM device info class
    /// </summary>
    /// <param name="info">SerialDeviceInfo structure from FireLibs.IO library</param>
    public class COMMDeviceInfo(SerialDeviceInfo info) : IDSRInputDeviceInfo
    {
        /// <inheritdoc/>
        public string Id => Info.PortName;
        /// <inheritdoc/>
        public string Name => $"Controller {Id}";

        /// <summary>
        /// SerialDeviceInfo structure from FireLibs.IO library
        /// </summary>
        public SerialDeviceInfo Info { get; private set; } = info;

        /// <inheritdoc/>
        public IDSRInputController CreateController()
        {
            return new COMM(this);
        }
    }
    /// <summary>
    /// COM controller class
    /// </summary>
    public class COMM : IDSRInputController
    {
        const BaudRates BaudRate = BaudRates.BR57600;
        private static readonly byte[] infoReportRequst = [0x00];
        private static readonly byte[] inputReportRequst = [0x01];

        private readonly SerialPort port;
        private COMInputReport rawReport = new();
        private readonly IDSRInputReport report;
        private COMInfoReport? information;
        private readonly COMOutputReport outReport = new();

        private readonly SixAxisProcess motPro = new();
        private readonly ExpMovingAverageVector3 gyroAvg = new();
        private DSRVector3 lastGyro = new();
        /// <inheritdoc/>
        public string Id { get; private set; }
        /// <inheritdoc/>
        public string Name => $"Controller {Id}";
        /// <inheritdoc/>
        public string Type => "COMM";
        /// <inheritdoc/>
        public bool IsConnected => port.IsConnected;
        /// <inheritdoc/>
        public string ImgPath => "COMM.png";

        private DSRLogger logger;
        /// <summary>
        /// Creates a COMM controller.
        /// COMM class constructor.
        /// </summary>
        /// <param name="info"></param>
        public COMM(COMMDeviceInfo info)
        {
            Id = info.Id;
            report = new DefaultDSRInputReport(6, 6, 32, 2,0,4,2);
            port = new(info.Info, BaudRate)
            {
                ReadTotalTimeoutConstant = 10
            };
            logger = DSRLogger.GetLogger($"DSRemapper.COMM<{Id}>");
        }
        /// <inheritdoc/>
        public void Connect()
        {
            if (!IsConnected)
            {
                port.Connect();
            }
        }

        /// <inheritdoc/>
        public void Disconnect()
        {
            if (IsConnected)
            {
                port.Disconnect();
            }
        }

        /// <inheritdoc/>
        public void Dispose()
        {
            Disconnect();
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Reads info report from COM controller
        /// </summary>
        /// <returns></returns>
        private COMInfoReport? ReadInfoReport()
        {
            port.CancelCurrentIO(tx: false);
            port.FlushRXBuffer();

            port.Write(infoReportRequst);
            if(port.Read(out COMInfoReport report)>=Marshal.SizeOf<COMInfoReport>())
                return report;

            return null;
        }

        /// <inheritdoc/>
        public IDSRInputReport GetInputReport()
        {
            information ??= ReadInfoReport();
            //logger.LogDebug($"{information?.AccelerometerScale.ToString() ?? "null"} | {information?.GyroscopeScale.ToString() ?? "null"}");

            port.FlushRXBuffer();

            port.Write(inputReportRequst);

            if (port.Read(out rawReport) >= Marshal.SizeOf<COMInputReport>())
            {
                report.Axes[0] = rawReport.Axes[0].ToFloatAxis();
                report.Axes[1] = rawReport.Axes[1].ToFloatAxis();
                report.Axes[2] = rawReport.Axes[2].ToFloatAxis();
                report.Axes[3] = rawReport.Axes[3].ToFloatAxis();
                report.Axes[4] = rawReport.Axes[4].ToFloatAxis();
                report.Axes[5] = rawReport.Axes[5].ToFloatAxis();

                report.Sliders[0] = rawReport.Sliders[0].ToFloatAxis();
                report.Sliders[1] = rawReport.Sliders[1].ToFloatAxis();
                report.Sliders[2] = rawReport.Sliders[2].ToFloatAxis();
                report.Sliders[3] = rawReport.Sliders[3].ToFloatAxis();
                report.Sliders[4] = rawReport.Sliders[4].ToFloatAxis();
                report.Sliders[5] = rawReport.Sliders[5].ToFloatAxis();

                for (int i = 0; i < report.Buttons.Length; i++)
                {
                    report.Buttons[i] = Convert.ToBoolean(rawReport.Buttons & (1 << i % 32));
                }

                report.Povs[0].Angle = rawReport.Pov0 == ushort.MaxValue ? -1 : rawReport.Pov0 / 100f;
                report.Povs[1].Angle = rawReport.Pov1 == ushort.MaxValue ? -1 : rawReport.Pov1 / 100f;

                if (information != null)
                {
                    if (information.Value.AccelerometerScale > 0 && information.Value.GyroscopeScale > 0)
                    {
                        float accelScale = 32768 / information.Value.AccelerometerScale;
                        float gyroScale = 32768 / information.Value.GyroscopeScale;
                        report.RawAccel = new DSRVector3(rawReport.Accelerometer[0] / accelScale,
                            rawReport.Accelerometer[1] / accelScale, rawReport.Accelerometer[2] / accelScale);
                        report.Gyro = new DSRVector3(rawReport.Gyroscope[0] / gyroScale,
                            rawReport.Gyroscope[1] / gyroScale, rawReport.Gyroscope[2] / gyroScale);

                        DSRVector3 temp = report.Gyro - lastGyro;
                        if (temp.Length < 1f)
                            gyroAvg.Update(report.Gyro, 200);

                        lastGyro = report.Gyro;

                        report.Gyro -= gyroAvg.Average;

                        motPro.Update(report.RawAccel, report.Gyro);

                        report.Grav = -motPro.Grav;
                        report.Accel = motPro.Accel;
                        report.Rotation = motPro.rotation;
                        report.DeltaRotation = motPro.deltaRotation;
                        //report.DeltaTime = motPro.DeltaTime;
                    }
                    else
                    {
                        report.RawAccel = new DSRVector3(rawReport.Accelerometer[0], rawReport.Accelerometer[1], rawReport.Accelerometer[2]);
                        report.Gyro = new DSRVector3(rawReport.Gyroscope[0], rawReport.Gyroscope[1], rawReport.Gyroscope[2]);
                    }
                }
                else
                {
                    report.RawAccel = new DSRVector3(rawReport.Accelerometer[0], rawReport.Accelerometer[1], rawReport.Accelerometer[2]);
                    report.Gyro = new DSRVector3(rawReport.Gyroscope[0], rawReport.Gyroscope[1], rawReport.Gyroscope[2]);
                }

                if (!float.IsNormal(report.RawAccel.Length))
                    report.RawAccel = new DSRVector3();
                if (!float.IsNormal(report.Gyro.Length))
                    report.Gyro = new DSRVector3();
                if (!float.IsNormal(report.Grav.Length))
                    report.Grav = new DSRVector3();
                if (!float.IsNormal(report.Accel.Length))
                    report.Accel = new DSRVector3();
            }

            return report;
        }

        /// <inheritdoc/>
        public void SendOutputReport(DefaultDSROutputReport report)
        {
            for (int i = 0; i < report.Rumble.Length; i++)
                outReport.Axes[i] = report.Rumble[i].ToShortAxis();
            int maxLeds = Math.Min(report.ExtLeds.Length, outReport.Buttons.Length);
            for (int i = 0; i < maxLeds; i++)
                outReport.Buttons[i] = (byte)report.ExtLeds[i];

            port.FlushRXBuffer();
            port.Write(outReport);
            port.Read(out _, 1); // makes a little delay using the timeout, or until the response byte is sent.
        }
    }

    /// <summary>
    /// Utils class for raw data conversion
    /// </summary>
    internal static class FloatExtensions
    {
        public static float ToFloatAxis(this short axis) => (float)axis / (short.MaxValue + (axis < 0 ? 1 : 0));
        public static short ToShortAxis(this float axis) => (short)(axis * (axis < 0 ? -short.MinValue : short.MaxValue));
        public static sbyte ToSByteAxis(this float axis) => (sbyte)(axis * (axis < 0 ? -sbyte.MinValue : sbyte.MaxValue));
        public static byte ToByteTrigger(this float axis) => (byte)(axis * byte.MaxValue);
    }
}