/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name         : Form1.cs
* Author            : Motion MEMS Application Team 
* Version           : 3.0.0.0
* Date              : 06 June 2012
* Description       : Unico Lite code
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

using System;
using System.Threading;
using System.Diagnostics;
using System.IO.Ports;
using System.Management;
using System.Reflection;
using System.Windows.Forms;

namespace Unico_Lite
{
    public partial class Form1 : Form
    {
        delegate void SetTextCallback(string text);

        #region Constants
        const int MAX_KITS = 4;                 //  Max numbers of kits supported

        public struct Kits                      //  Structure that describe the kit
        {
            public string Name;                 //  STEVAL-CODE 
            public int words;                   //  Number of bytes to acquire (please refer to UM2116 - STEVAL-MKI109V3)
            public string setdb;                //  set evaluation board code
        }
        #endregion

        #region Variables
        Kits[] MKI = new Kits[MAX_KITS];        //  Kits array
        int Kit_Index = -1;                     //  Default Combo Box value for Kit Selection
        string DataReadFromSerialPort = "";     //  Temporary string for Serial Port reading
        bool Start = false;                     //  Flag to monitor if the Start button has been pressed or not
        volatile bool Connected = false;        //  Flag to monitor connection status: volatile because used in different threads
        public string readACC = "*r";           //  Read instriction prefix for Accelerometer sensor
        public string writeACC = "*w";          //  Write instriction prefix for Accelerometer sensor
        public string readGYR = "*gr";          //  Read instriction prefix for Gyroscope sensor
        public string writeGYR = "*gw";         //  Write instriction prefix for Gyroscope sensor
        public string readMAG = "*mr";          //  Read instriction prefix for Magnetometer sensor
        public string writeMAG = "*mw";         //  Write instriction prefix for Magnetometer sensor
        public string readPRS = "*pr";          //  Read instriction prefix for Pressure sensor
        public string writePRS = "*pw";         //  Write instriction prefix for Pressure sensor
        #endregion 

        #region Classes
        public SerialPort c_Serial;             //  .Net Serial port class to comunicate with the evaluation board
        #endregion

        #region Initialization
        public Form1()
        {
            InitializeComponent();

            //  Get file version
            LB_VersionValue.Text = GetFileVersion();

            //  Fill the evaluation board array
            //  STEVAL-MKI178V1 for LSM6DSL Combo (accelerometer and gyroscope) sensor
            MKI[0].Name = "MKI178V1 (LSM6DSL)";             
            MKI[0].words = 22;
            MKI[0].setdb = "*setdb178v1";

            //  STEVAL-MKI179V1 for LIS2DW12 accelerometer sensor
            MKI[1].Name = "MKI179V1 (LIS2DW12)";
            MKI[1].words = 13;           
            MKI[1].setdb = "*setdb179v1";

            //  STEVAL-MKI181V1 for LIS2MDL magnetometer sensor
            MKI[2].Name = "MKI181V1 (LIS2MDL)";
            MKI[2].words = 12;           
            MKI[2].setdb = "*setdb181v1";

            //  STEVAL-MET001V1 for LPS22HB pressure sensor
            MKI[3].Name = "MET001V1 (LPS22HB)";
            MKI[3].words = 14;
            MKI[3].setdb = "*setdb001V1";

            //  Fill the Combo Box with Kits
            for (int i = 0; i < MAX_KITS; i++)
            {
                CBX_Kit.Items.Add(MKI[i].Name);
            }

            //  Initialize Serial Port Objects
            c_Serial = new SerialPort();                    
            DetectCOMPorts();
        }

        private string GetFileVersion()
        {
            Assembly assembly = Assembly.GetExecutingAssembly();
            FileVersionInfo fvi = FileVersionInfo.GetVersionInfo(assembly.Location);
            return fvi.FileVersion;
        }
        #endregion

        #region Connection
        private void DetectCOMPorts()
        {
            ManagementScope connectionScope = new ManagementScope();
            SelectQuery serialQuery = new SelectQuery("SELECT * FROM Win32_SerialPort");
            ManagementObjectSearcher searcher = new ManagementObjectSearcher(connectionScope, serialQuery);

            // Fill Combo Box COM Ports
            CBX_ComPorts.Items.Clear();
            CBX_ComPorts.Text = "";
            string[] COM_Ports = SerialPort.GetPortNames();
            Array.Sort(COM_Ports, StringComparer.InvariantCulture);
            for (int i = 0; i < COM_Ports.Length; i++)
            {
                CBX_ComPorts.Items.Add(COM_Ports[i]);
            }

            try
            {
                // Autoselect first "STMicroelectronics Virtual COM Port" if found
                foreach (ManagementObject item in searcher.Get())
                {
                    string COM_Description = item["Description"].ToString();
                    if (COM_Description.Contains("STMicroelectronics Virtual COM Port"))
                    {
                        CBX_ComPorts.Text = item["DeviceID"].ToString();
                        break;
                    }
                }
            }
            catch
            {
                MessageBox.Show("Cannot detect COM ports", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BTN_ComRefresh_Click(object sender, EventArgs e)
        {
            DetectCOMPorts();
        }

        private void BTN_Connect_Click(object sender, EventArgs e)
        {
            if (!Connected)
            {
                if (CBX_Kit.SelectedIndex == -1)
                {
                    MessageBox.Show("Please select a KIT Name.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                }
                else
                {
                    if (CBX_ComPorts.SelectedIndex == -1)
                    {
                        MessageBox.Show("Please select a valid COM Port", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    }
                    else
                    {
                        // Initialize COM Port parameters
                        c_Serial.PortName = CBX_ComPorts.Text;
                        c_Serial.BaudRate = 115200;                                                         //  Assign COM Port Baud Rate 
                        c_Serial.DataBits = 8;                                                              //  Assign COM Port DataBits 
                        c_Serial.ReadTimeout = 500;                                                         //  Assign COM Port ReadTimeout 
                        c_Serial.WriteTimeout = 500;                                                        //  Assign COM Port WriteTimeout 
                        c_Serial.DataReceived += new SerialDataReceivedEventHandler(USB_DataReceived);      //  Assign COM Port Receive data Event 

                        try
                        {
                            c_Serial.Open();                                                //  Open the serial port

                            c_Serial.WriteLine(MKI[CBX_Kit.SelectedIndex].setdb + "\r\n");  //  Send the "setdb" command to the microcontroller just to configure itself for the right sensor 

                            Thread.Sleep(50);                                               //  Wait 50ms between serial port writings

                            c_Serial.WriteLine("*zoff\r\n");                                //  Send the "zoff" command

                            GB_DirectCommunication.Enabled = true;

                            CBX_Kit.Enabled = false;                                        //  Disable Combo Box Kit selection
                            CBX_ComPorts.Enabled = false;                                   //  Disable Combo Box COM selection
                            BTN_ComRefresh.Enabled = false;                                 //  Disable Button COM refresh

                            // Define Default RadioButton Selected
                            if (MKI[CBX_Kit.SelectedIndex].Name.Contains("MKI178V1"))
                            {
                                RB_Acc.Enabled = true; RB_Gyro.Enabled = true; RB_Mag.Enabled = false; RB_Prs.Enabled = false;
                                RB_Acc.Checked = true; RB_Gyro.Checked = false; RB_Mag.Checked = false; RB_Prs.Checked = false;
                                GB_DataOutput.Enabled = true;
                            }
                            else if (MKI[CBX_Kit.SelectedIndex].Name.Contains("MKI179V1"))
                            {
                                RB_Acc.Enabled = true; RB_Gyro.Enabled = false; RB_Mag.Enabled = false; RB_Prs.Enabled = false;
                                RB_Acc.Checked = true; RB_Gyro.Checked = false; RB_Mag.Checked = false; RB_Prs.Checked = false;
                                GB_DataOutput.Enabled = true;
                            }
                            else if (MKI[CBX_Kit.SelectedIndex].Name.Contains("MKI181V1"))
                            {
                                RB_Acc.Enabled = false; RB_Gyro.Enabled = false; RB_Mag.Enabled = true; RB_Prs.Enabled = false;
                                RB_Acc.Checked = false; RB_Gyro.Checked = false; RB_Mag.Checked = true; RB_Prs.Checked = false;
                                GB_DataOutput.Enabled = true;
                            }
                            else if (MKI[CBX_Kit.SelectedIndex].Name.Contains("MET001V1"))
                            {
                                RB_Acc.Enabled = false; RB_Gyro.Enabled = false; RB_Mag.Enabled = false; RB_Prs.Enabled = true;
                                RB_Acc.Checked = false; RB_Gyro.Checked = false; RB_Mag.Checked = false; RB_Prs.Checked = true;
                                GB_DataOutput.Enabled = false; // Only register read/write for Pressure sensor Case (X,Y,Z not existing)
                            }

                            Connected = true;
                            BTN_Connect.Text = "Disconnect";
                        }
                        catch
                        {
                            MessageBox.Show("Impossible to open the COM Port selected", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                        }
                    }
                }                
            }
            else
            {
                // Erase connection status
                Connected = false;
                BTN_Connect.Text = "Connect";

                // Erase start status
                Start = false;
                BTN_Start.Text = "Start";

                c_Serial.DataReceived -= USB_DataReceived;                          //  Remove COM Port Receive data Event 
                c_Serial.WriteLine("*stop\r\n");                                    //  Send the "stop" command
                Thread.Sleep(10);                                                   //  Wait 10ms between serial port writings
                c_Serial.WriteLine("*zon\r\n");                                     //  Send the "zon" command
                Thread.Sleep(10);                                                   //  Wait 10ms between serial port writings
                c_Serial.WriteLine("*dbreset\r\n");                                 //  Send the "dbreset" command
                Thread thread = new Thread(new ThreadStart(CloseSerialPort));       //  Close the serial port in different thread to avoid deadlock
                thread.Start();

                CBX_Kit.Enabled = true;                                             //  Enable Combo Box Kit selection
                CBX_ComPorts.Enabled = true;                                        //  Enable Combo Box COM selection
                BTN_ComRefresh.Enabled = true;                                      //  Enable Button COM refresh

                RB_Acc.Enabled = false;                                             //  Disable Accelerometer RadioButton
                RB_Gyro.Enabled = false;                                            //  Disable Gyroscope RadioButton
                RB_Mag.Enabled = false;                                             //  Disable Magnetometer RadioButton
                RB_Prs.Enabled = false;                                             //  Disable Pressure RadioButton

                RB_Acc.Checked = false;                                             //  Uncheck Accelerometer RadioButton
                RB_Gyro.Checked = false;                                            //  Uncheck Gyroscope RadioButton
                RB_Mag.Checked = false;                                             //  Uncheck Magnetometer RadioButton
                RB_Prs.Checked = false;                                             //  Uncheck Pressure RadioButton

                GB_DirectCommunication.Enabled = false;                             //  Disable Direct Communication Group Box
                GB_DataOutput.Enabled = false;                                      //  Disable Continuous Data Output Group Box

                TB_Address.Clear();                                                 //  Clear Address Text Box content
                TB_Value.Clear();                                                   //  Clear Value Text Box content
                TB_Val1.Clear();                                                    //  Clear Axis1 Text Box content
                TB_Val2.Clear();                                                    //  Clear Axis2 Text Box content
                TB_Val3.Clear();                                                    //  Clear Axis3 Text Box content
            }            
        }

        private void BTN_Start_Click(object sender, EventArgs e)
        {
            if (!Start)
            {
                c_Serial.WriteLine("*start\r\n");                                   //  Send the "start" command

                Start = true;
                BTN_Start.Text = "Stop";
            }
            else
            {
                c_Serial.WriteLine("*stop\r\n");                                    //  Send the "stop" command

                Start = false;
                BTN_Start.Text = "Start";
            }
            
        }

        private void CloseSerialPort()
        {
            try
            {
                c_Serial.Close();
            }
            catch
            {
                MessageBox.Show("Cannot close serial port.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }
        #endregion

        #region Decod
        private void String_Vector_Decod()
        {
            int Character;
            char[] Tvect = new char[30];
            byte[] Ivect = new byte[30];
            int N_Strings = 0;
            byte[] Buffer_Input = new byte[100000];
            byte[,] Buffer_Output = new byte[5000, 30];
            int N_Bytes;

            Int16 AX1 = 0;
            Int16 AX2 = 0;
            Int16 AX3 = 0;
            
            try
            {
                Character = c_Serial.ReadChar();
            }
            catch
            {
                Character = 0;
            }

            //  Check if "st" start characters are received
            if (Character == 116)                                                           
            {
                //  Read all data in the buffer
                N_Bytes = c_Serial.BytesToRead;
                c_Serial.Read(Buffer_Input, 0, N_Bytes);

                //  Format the data input
                Manage_Input_Buffer(Buffer_Input, Buffer_Output, N_Bytes, out N_Strings);    

                for (int j = 0; j < N_Strings; j++)
                {
                    for (int i = 0; i < MKI[Kit_Index].words; i++)
                    {
                        Ivect[i] = Buffer_Output[j, i];
                    }

                    //  STEVAL-MKI178V1 for LSM6DSL accelerometer sensor
                    if ((MKI[Kit_Index].Name.Contains("MKI178V1")) && (RB_Acc.Checked))
                    {
                        AX1 = BitConverter.ToInt16(new byte[2] { Ivect[1], Ivect[0] }, 0);
                        AX2 = BitConverter.ToInt16(new byte[2] { Ivect[3], Ivect[2] }, 0);
                        AX3 = BitConverter.ToInt16(new byte[2] { Ivect[5], Ivect[4] }, 0);                        
                    }

                    //  STEVAL-MKI178V1 for LSM6DSL gyroscope sensor
                    if ((MKI[Kit_Index].Name.Contains("MKI178V1")) && (RB_Gyro.Checked))
                    {
                        AX1 = BitConverter.ToInt16(new byte[2] { Ivect[7], Ivect[6] }, 0);
                        AX2 = BitConverter.ToInt16(new byte[2] { Ivect[9], Ivect[8] }, 0);
                        AX3 = BitConverter.ToInt16(new byte[2] { Ivect[11], Ivect[10] }, 0);
                    }

                    //  STEVAL-MKI179V1 for LIS2DW12 accelerometer sensor
                    if ((MKI[Kit_Index].Name.Contains("MKI179V1")) && (RB_Acc.Checked))
                    {
                        AX1 = BitConverter.ToInt16(new byte[2] { Ivect[1], Ivect[0] }, 0);
                        AX2 = BitConverter.ToInt16(new byte[2] { Ivect[3], Ivect[2] }, 0);
                        AX3 = BitConverter.ToInt16(new byte[2] { Ivect[5], Ivect[4] }, 0);
                    }

                    //  STEVAL-MKI181V1 for LIS2MDL magnetometer sensor
                    if ((MKI[Kit_Index].Name.Contains("MKI181V1")) && (RB_Mag.Checked))
                    {
                        AX1 = BitConverter.ToInt16(new byte[2] { Ivect[1], Ivect[0] }, 0);
                        AX2 = BitConverter.ToInt16(new byte[2] { Ivect[3], Ivect[2] }, 0);
                        AX3 = BitConverter.ToInt16(new byte[2] { Ivect[5], Ivect[4] }, 0);
                    }

                    //  Show data in the textboxes (be careful for cross-thread issue)
                    try
                    {
                        if (TB_Val1.InvokeRequired) { SetTextCallback d = new SetTextCallback(SetText1); Invoke(d, new object[] { Convert.ToString(AX1) }); } else { TB_Val1.Text = Convert.ToString(AX1); }
                        if (TB_Val2.InvokeRequired) { SetTextCallback d = new SetTextCallback(SetText2); Invoke(d, new object[] { Convert.ToString(AX2) }); } else { TB_Val2.Text = Convert.ToString(AX2); }
                        if (TB_Val3.InvokeRequired) { SetTextCallback d = new SetTextCallback(SetText3); Invoke(d, new object[] { Convert.ToString(AX3) }); } else { TB_Val3.Text = Convert.ToString(AX3); }
                    }
                    catch
                    {
                    }
                }
            }
        }

        private void Manage_Input_Buffer(byte[] Buffer_RND, byte[,] Buffer_ORD, int N_Bytes, out int Numero)
        {
            int n_temp = 1;

            //  First line
            for (int i = 0; i < MKI[Kit_Index].words; i++)
            {
                Buffer_ORD[0, i] = Buffer_RND[i];
            }
            N_Bytes -= MKI[Kit_Index].words;

            //  Other lines
            long u = 0;  //  Number of elements
            long k = 1;  //  Number of words

            while (N_Bytes >= MKI[Kit_Index].words + 4)
            {
                if (Buffer_RND[u++] == 115)             //  's'
                {
                    if (Buffer_RND[u++] == 116)         //  't'
                    {
                        for (int i = 0; i < MKI[Kit_Index].words; i++)
                        {
                            Buffer_ORD[k, i] = Buffer_RND[u++];
                        }
                        N_Bytes -= MKI[Kit_Index].words + 4;
                        k++;
                        n_temp++;
                    }
                }

                if (u >= 100000)
                    break;
            }


            Numero = n_temp;
        }
        
        private void SetText1(string text)
        {
            if (Connected)
                TB_Val1.Text = text;
        }

        private void SetText2(string text)
        {
            if (Connected)
                TB_Val2.Text = text;
        }

        private void SetText3(string text)
        {
            if (Connected)
                TB_Val3.Text = text;
        }
        #endregion

        #region Events
        void USB_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            int Character;

            try
            {
                Character = c_Serial.ReadChar();                                        //  Reads one char from the serial port
            }
            catch
            {
                Character = 0;
            }

            if (Character == 115)
            {
                String_Vector_Decod();                                                  //  If the character is equal to 115 ('s') the software starts to decode the string
            }
        }

        private void CBX_Kit_SelectedIndexChanged(object sender, EventArgs e)
        {
            Kit_Index = CBX_Kit.SelectedIndex;
        }
        #endregion

        #region Buttons
        private void BTN_Read_Click(object sender, EventArgs e)
        {
            bool ValidValue = false;
            string Add = TB_Address.Text;
            long output = 0;

            // Check address validity
            ValidValue = long.TryParse(Add, System.Globalization.NumberStyles.HexNumber, null, out output);
            ValidValue &= (Add.Length == 2);

            if (ValidValue)
            {
                string SensorRead = "";

                if (Start)
                {
                    c_Serial.WriteLine("*stop\r\n");                                        //  Send the "stop" command
                    Thread.Sleep(10);
                }

                DataReadFromSerialPort = c_Serial.ReadExisting();                           //  Read possible garbage on serial port

                if (RB_Acc.Checked) { SensorRead = readACC; }
                else if (RB_Gyro.Checked) { SensorRead = readGYR; }
                else if (RB_Mag.Checked) { SensorRead = readMAG; }
                else if (RB_Prs.Checked) { SensorRead = readPRS; }

                c_Serial.WriteLine(SensorRead + TB_Address.Text + "\r\n");                  //  Send the "read" command

                Thread.Sleep(10);

                DataReadFromSerialPort = c_Serial.ReadExisting();                           //  Read the reply message on the Serial Port

                if (DataReadFromSerialPort != "")                                           //  Decode the data
                {
                    string[] Params = DataReadFromSerialPort.Split('h');
                    TB_Value.Text = Params[1];
                }

                if (Start)
                {
                    c_Serial.WriteLine("*start\r\n");                                       //  Restart comunication in the Serial Port
                    Thread.Sleep(10);
                }
            }
            else
            {
                MessageBox.Show("Invalid hexadecimal address value.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void BTN_Write_Click(object sender, EventArgs e)
        {
            bool ValidValue = false;
            string Add = TB_Address.Text;
            string Val = TB_Value.Text;
            long output = 0;

            // Check address / value validity
            ValidValue = long.TryParse(Add, System.Globalization.NumberStyles.HexNumber, null, out output);
            ValidValue &= long.TryParse(Val, System.Globalization.NumberStyles.HexNumber, null, out output);
            ValidValue &= (Add.Length == 2);
            ValidValue &= (Val.Length == 2);

            if (ValidValue)
            {
                string SensorWrite = "";

                if (Start)
                {
                    c_Serial.WriteLine("*stop\r\n");                                        //  Send the "stop" command
                    Thread.Sleep(10);
                }

                if (RB_Acc.Checked) { SensorWrite = writeACC; }
                else if (RB_Gyro.Checked) { SensorWrite = writeGYR; }
                else if (RB_Mag.Checked) { SensorWrite = writeMAG; }
                else if (RB_Prs.Checked) { SensorWrite = writePRS; }

                c_Serial.WriteLine(SensorWrite + Add + Val + "\r\n");                       //  Send the "write" command

                if (Start)
                {
                    c_Serial.WriteLine("*start\r\n");                                       //  Restart comunication in the Serial Port
                    Thread.Sleep(10);
                }
            }
            else
            {
                MessageBox.Show("Invalid hexadecimal values.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            
        }
        #endregion
    }
}
