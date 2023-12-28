using DSRemapper;
using DSRemapper.Core;
using DSRemapper.DSRMath;
using DSRemapper.SixAxis;
using DSRemapper.Types;
using FireLibs.IO.COMPorts.Win;
using SerialDeviceInfo = FireLibs.IO.COMPorts.SerialDeviceInfo;
using System.Runtime.InteropServices;

namespace DSRemapper.COMM
{
    /// <summary>
    /// Structure that match COM controller info data structure
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 32)]
    internal struct COMInfoReport
    {
        public byte Axis = 0; // reserved for the future (useless at the moment)
        public byte Buttons = 0; // reserved for the future (useless at the moment)
        public byte Povs = 0; // reserved for the future (useless at the moment)
        public ushort AccelScale = 0;
        public ushort GyroScale = 0;

        public COMInfoReport() { }
    }

    /// <summary>
    /// Structure that match COM controller input data structure
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 64)]
    internal struct COMInputReport
    {
        public byte id = 0; // reserved for the future (useless at the moment)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 12)]
        public short[] Axis = new short[12];
        public uint Buttons = 0;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public ushort[] Pov = new ushort[2];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public short[] Accel = new short[3];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public short[] Gyro = new short[3];

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
        public byte id = 0; // reserved for the future (useless at the moment)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public short[] Motors = new short[6];
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public byte[] Leds = new byte[6];

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

            port.FlushRXBuffer();

            port.Write(inputReportRequst);

            if (port.Read(out rawReport) >= Marshal.SizeOf<COMInputReport>())
            { 
                report.Axes[0] = rawReport.Axis[0].ToFloatAxis();
                report.Axes[1] = rawReport.Axis[1].ToFloatAxis();
                report.Axes[2] = rawReport.Axis[2].ToFloatAxis();
                report.Axes[3] = rawReport.Axis[3].ToFloatAxis();
                report.Axes[4] = rawReport.Axis[4].ToFloatAxis();
                report.Axes[5] = rawReport.Axis[5].ToFloatAxis();

                report.Sliders[0] = rawReport.Axis[6].ToFloatAxis();
                report.Sliders[1] = rawReport.Axis[7].ToFloatAxis();
                report.Sliders[2] = rawReport.Axis[8].ToFloatAxis();
                report.Sliders[3] = rawReport.Axis[9].ToFloatAxis();
                report.Sliders[4] = rawReport.Axis[10].ToFloatAxis();
                report.Sliders[5] = rawReport.Axis[11].ToFloatAxis();

                for (int i = 0; i < report.Buttons.Length; i++)
                {
                    report.Buttons[i] = Convert.ToBoolean(rawReport.Buttons & (1 << i % 32));
                }

                report.Povs[0].Angle = rawReport.Pov[0] == ushort.MaxValue ? -1 : rawReport.Pov[0] / 100f;
                report.Povs[1].Angle = rawReport.Pov[1] == ushort.MaxValue ? -1 : rawReport.Pov[1] / 100f;

                if (information != null)
                {
                    if (information.Value.AccelScale > 0 && information.Value.GyroScale > 0)
                    {
                        float accelScale = 32768 / information.Value.AccelScale;
                        float gyroScale = 32768 / information.Value.GyroScale;
                        report.RawAccel = new DSRVector3(rawReport.Accel[0] / accelScale,
                            rawReport.Accel[1] / accelScale, rawReport.Accel[2] / accelScale);
                        report.Gyro = new DSRVector3(rawReport.Gyro[0] / gyroScale,
                            rawReport.Gyro[1] / gyroScale, rawReport.Gyro[2] / gyroScale);

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
                        report.RawAccel = new DSRVector3(rawReport.Accel[0], rawReport.Accel[1], rawReport.Accel[2]);
                        report.Gyro = new DSRVector3(rawReport.Gyro[0], rawReport.Gyro[1], rawReport.Gyro[2]);
                    }
                }
                else
                {
                    report.RawAccel = new DSRVector3(rawReport.Accel[0], rawReport.Accel[1], rawReport.Accel[2]);
                    report.Gyro = new DSRVector3(rawReport.Gyro[0], rawReport.Gyro[1], rawReport.Gyro[2]);
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
                outReport.Motors[i] = report.Rumble[i].ToShortAxis();
            for (int i = 0; i < report.ExtLeds.Length; i++)
                outReport.Leds[i] = (byte)report.ExtLeds[i];

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