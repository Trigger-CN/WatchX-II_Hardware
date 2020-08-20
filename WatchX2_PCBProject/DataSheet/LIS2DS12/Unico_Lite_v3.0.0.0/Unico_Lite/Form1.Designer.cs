namespace Unico_Lite
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.label2 = new System.Windows.Forms.Label();
            this.CBX_Kit = new System.Windows.Forms.ComboBox();
            this.BTN_Start = new System.Windows.Forms.Button();
            this.BTN_Connect = new System.Windows.Forms.Button();
            this.GB_DirectCommunication = new System.Windows.Forms.GroupBox();
            this.BTN_Write = new System.Windows.Forms.Button();
            this.BTN_Read = new System.Windows.Forms.Button();
            this.label8 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.TB_Value = new System.Windows.Forms.TextBox();
            this.TB_Address = new System.Windows.Forms.TextBox();
            this.GB_DataOutput = new System.Windows.Forms.GroupBox();
            this.label10 = new System.Windows.Forms.Label();
            this.TB_Val3 = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.RB_Prs = new System.Windows.Forms.RadioButton();
            this.RB_Mag = new System.Windows.Forms.RadioButton();
            this.TB_Val2 = new System.Windows.Forms.TextBox();
            this.RB_Gyro = new System.Windows.Forms.RadioButton();
            this.RB_Acc = new System.Windows.Forms.RadioButton();
            this.label9 = new System.Windows.Forms.Label();
            this.TB_Val1 = new System.Windows.Forms.TextBox();
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.BTN_ComRefresh = new System.Windows.Forms.Button();
            this.CBX_ComPorts = new System.Windows.Forms.ComboBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.LB_VersionValue = new System.Windows.Forms.Label();
            this.LB_Version = new System.Windows.Forms.Label();
            this.GB_DirectCommunication.SuspendLayout();
            this.GB_DataOutput.SuspendLayout();
            this.groupBox3.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).BeginInit();
            this.SuspendLayout();
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(6, 21);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(98, 13);
            this.label2.TabIndex = 7;
            this.label2.Text = "Select STEVAL kit:";
            // 
            // CBX_Kit
            // 
            this.CBX_Kit.FormattingEnabled = true;
            this.CBX_Kit.Location = new System.Drawing.Point(6, 37);
            this.CBX_Kit.Name = "CBX_Kit";
            this.CBX_Kit.Size = new System.Drawing.Size(144, 21);
            this.CBX_Kit.TabIndex = 6;
            this.CBX_Kit.SelectedIndexChanged += new System.EventHandler(this.CBX_Kit_SelectedIndexChanged);
            // 
            // BTN_Start
            // 
            this.BTN_Start.Location = new System.Drawing.Point(221, 70);
            this.BTN_Start.Name = "BTN_Start";
            this.BTN_Start.Size = new System.Drawing.Size(65, 23);
            this.BTN_Start.TabIndex = 3;
            this.BTN_Start.Text = "Start";
            this.BTN_Start.UseVisualStyleBackColor = true;
            this.BTN_Start.Click += new System.EventHandler(this.BTN_Start_Click);
            // 
            // BTN_Connect
            // 
            this.BTN_Connect.Location = new System.Drawing.Point(6, 107);
            this.BTN_Connect.Name = "BTN_Connect";
            this.BTN_Connect.Size = new System.Drawing.Size(144, 23);
            this.BTN_Connect.TabIndex = 2;
            this.BTN_Connect.Text = "Connect";
            this.BTN_Connect.UseVisualStyleBackColor = true;
            this.BTN_Connect.Click += new System.EventHandler(this.BTN_Connect_Click);
            // 
            // GB_DirectCommunication
            // 
            this.GB_DirectCommunication.Controls.Add(this.BTN_Write);
            this.GB_DirectCommunication.Controls.Add(this.BTN_Read);
            this.GB_DirectCommunication.Controls.Add(this.label8);
            this.GB_DirectCommunication.Controls.Add(this.label7);
            this.GB_DirectCommunication.Controls.Add(this.label6);
            this.GB_DirectCommunication.Controls.Add(this.label5);
            this.GB_DirectCommunication.Controls.Add(this.label4);
            this.GB_DirectCommunication.Controls.Add(this.label3);
            this.GB_DirectCommunication.Controls.Add(this.TB_Value);
            this.GB_DirectCommunication.Controls.Add(this.TB_Address);
            this.GB_DirectCommunication.Enabled = false;
            this.GB_DirectCommunication.Location = new System.Drawing.Point(174, 60);
            this.GB_DirectCommunication.Name = "GB_DirectCommunication";
            this.GB_DirectCommunication.Size = new System.Drawing.Size(296, 83);
            this.GB_DirectCommunication.TabIndex = 2;
            this.GB_DirectCommunication.TabStop = false;
            this.GB_DirectCommunication.Text = "Direct Communication";
            // 
            // BTN_Write
            // 
            this.BTN_Write.Location = new System.Drawing.Point(221, 48);
            this.BTN_Write.Name = "BTN_Write";
            this.BTN_Write.Size = new System.Drawing.Size(65, 23);
            this.BTN_Write.TabIndex = 9;
            this.BTN_Write.Text = "Write";
            this.BTN_Write.UseVisualStyleBackColor = true;
            this.BTN_Write.Click += new System.EventHandler(this.BTN_Write_Click);
            // 
            // BTN_Read
            // 
            this.BTN_Read.Location = new System.Drawing.Point(221, 19);
            this.BTN_Read.Name = "BTN_Read";
            this.BTN_Read.Size = new System.Drawing.Size(65, 23);
            this.BTN_Read.TabIndex = 8;
            this.BTN_Read.Text = "Read";
            this.BTN_Read.UseVisualStyleBackColor = true;
            this.BTN_Read.Click += new System.EventHandler(this.BTN_Read_Click);
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(120, 37);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(37, 13);
            this.label8.TabIndex = 7;
            this.label8.Text = "Value:";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(8, 37);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(48, 13);
            this.label7.TabIndex = 6;
            this.label7.Text = "Address:";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(197, 37);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(13, 13);
            this.label6.TabIndex = 5;
            this.label6.Text = "h";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(97, 37);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(13, 13);
            this.label5.TabIndex = 4;
            this.label5.Text = "h";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(154, 37);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(18, 13);
            this.label4.TabIndex = 3;
            this.label4.Text = "0x";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(54, 37);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(18, 13);
            this.label3.TabIndex = 2;
            this.label3.Text = "0x";
            // 
            // TB_Value
            // 
            this.TB_Value.Location = new System.Drawing.Point(172, 34);
            this.TB_Value.MaxLength = 2;
            this.TB_Value.Name = "TB_Value";
            this.TB_Value.Size = new System.Drawing.Size(25, 20);
            this.TB_Value.TabIndex = 1;
            this.TB_Value.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // TB_Address
            // 
            this.TB_Address.Location = new System.Drawing.Point(72, 34);
            this.TB_Address.MaxLength = 2;
            this.TB_Address.Name = "TB_Address";
            this.TB_Address.Size = new System.Drawing.Size(25, 20);
            this.TB_Address.TabIndex = 0;
            this.TB_Address.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // GB_DataOutput
            // 
            this.GB_DataOutput.Controls.Add(this.label10);
            this.GB_DataOutput.Controls.Add(this.TB_Val3);
            this.GB_DataOutput.Controls.Add(this.label12);
            this.GB_DataOutput.Controls.Add(this.RB_Prs);
            this.GB_DataOutput.Controls.Add(this.BTN_Start);
            this.GB_DataOutput.Controls.Add(this.RB_Mag);
            this.GB_DataOutput.Controls.Add(this.TB_Val2);
            this.GB_DataOutput.Controls.Add(this.RB_Gyro);
            this.GB_DataOutput.Controls.Add(this.RB_Acc);
            this.GB_DataOutput.Controls.Add(this.label9);
            this.GB_DataOutput.Controls.Add(this.TB_Val1);
            this.GB_DataOutput.Enabled = false;
            this.GB_DataOutput.Location = new System.Drawing.Point(174, 149);
            this.GB_DataOutput.Name = "GB_DataOutput";
            this.GB_DataOutput.Size = new System.Drawing.Size(296, 101);
            this.GB_DataOutput.TabIndex = 10;
            this.GB_DataOutput.TabStop = false;
            this.GB_DataOutput.Text = "Continous Data Output (LSB)";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(147, 53);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(42, 13);
            this.label10.TabIndex = 16;
            this.label10.Text = "Axis #3";
            // 
            // TB_Val3
            // 
            this.TB_Val3.Location = new System.Drawing.Point(150, 71);
            this.TB_Val3.Name = "TB_Val3";
            this.TB_Val3.Size = new System.Drawing.Size(65, 20);
            this.TB_Val3.TabIndex = 15;
            this.TB_Val3.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(76, 53);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(42, 13);
            this.label12.TabIndex = 14;
            this.label12.Text = "Axis #2";
            // 
            // RB_Prs
            // 
            this.RB_Prs.AutoSize = true;
            this.RB_Prs.Enabled = false;
            this.RB_Prs.Location = new System.Drawing.Point(240, 25);
            this.RB_Prs.Name = "RB_Prs";
            this.RB_Prs.Size = new System.Drawing.Size(47, 17);
            this.RB_Prs.TabIndex = 14;
            this.RB_Prs.TabStop = true;
            this.RB_Prs.Text = "PRS";
            this.RB_Prs.UseVisualStyleBackColor = true;
            // 
            // RB_Mag
            // 
            this.RB_Mag.AutoSize = true;
            this.RB_Mag.Enabled = false;
            this.RB_Mag.Location = new System.Drawing.Point(164, 25);
            this.RB_Mag.Name = "RB_Mag";
            this.RB_Mag.Size = new System.Drawing.Size(49, 17);
            this.RB_Mag.TabIndex = 13;
            this.RB_Mag.TabStop = true;
            this.RB_Mag.Text = "MAG";
            this.RB_Mag.UseVisualStyleBackColor = true;
            // 
            // TB_Val2
            // 
            this.TB_Val2.Location = new System.Drawing.Point(79, 71);
            this.TB_Val2.Name = "TB_Val2";
            this.TB_Val2.Size = new System.Drawing.Size(65, 20);
            this.TB_Val2.TabIndex = 13;
            this.TB_Val2.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // RB_Gyro
            // 
            this.RB_Gyro.AutoSize = true;
            this.RB_Gyro.Enabled = false;
            this.RB_Gyro.Location = new System.Drawing.Point(88, 25);
            this.RB_Gyro.Name = "RB_Gyro";
            this.RB_Gyro.Size = new System.Drawing.Size(56, 17);
            this.RB_Gyro.TabIndex = 12;
            this.RB_Gyro.TabStop = true;
            this.RB_Gyro.Text = "GYRO";
            this.RB_Gyro.UseVisualStyleBackColor = true;
            // 
            // RB_Acc
            // 
            this.RB_Acc.AutoSize = true;
            this.RB_Acc.Enabled = false;
            this.RB_Acc.Location = new System.Drawing.Point(12, 25);
            this.RB_Acc.Name = "RB_Acc";
            this.RB_Acc.Size = new System.Drawing.Size(46, 17);
            this.RB_Acc.TabIndex = 11;
            this.RB_Acc.TabStop = true;
            this.RB_Acc.Text = "ACC";
            this.RB_Acc.UseVisualStyleBackColor = true;
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(5, 53);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(42, 13);
            this.label9.TabIndex = 11;
            this.label9.Text = "Axis #1";
            // 
            // TB_Val1
            // 
            this.TB_Val1.Location = new System.Drawing.Point(8, 71);
            this.TB_Val1.Name = "TB_Val1";
            this.TB_Val1.Size = new System.Drawing.Size(65, 20);
            this.TB_Val1.TabIndex = 10;
            this.TB_Val1.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.BTN_ComRefresh);
            this.groupBox3.Controls.Add(this.CBX_ComPorts);
            this.groupBox3.Controls.Add(this.label11);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.BTN_Connect);
            this.groupBox3.Controls.Add(this.CBX_Kit);
            this.groupBox3.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.groupBox3.Location = new System.Drawing.Point(12, 112);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Size = new System.Drawing.Size(156, 138);
            this.groupBox3.TabIndex = 15;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "Connection Information";
            // 
            // BTN_ComRefresh
            // 
            this.BTN_ComRefresh.BackgroundImage = global::Unico_Lite.Properties.Resources.com_refresh_icon;
            this.BTN_ComRefresh.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.BTN_ComRefresh.Location = new System.Drawing.Point(128, 79);
            this.BTN_ComRefresh.Name = "BTN_ComRefresh";
            this.BTN_ComRefresh.Size = new System.Drawing.Size(23, 23);
            this.BTN_ComRefresh.TabIndex = 16;
            this.BTN_ComRefresh.UseVisualStyleBackColor = true;
            this.BTN_ComRefresh.Click += new System.EventHandler(this.BTN_ComRefresh_Click);
            // 
            // CBX_ComPorts
            // 
            this.CBX_ComPorts.FormattingEnabled = true;
            this.CBX_ComPorts.Location = new System.Drawing.Point(6, 80);
            this.CBX_ComPorts.Name = "CBX_ComPorts";
            this.CBX_ComPorts.Size = new System.Drawing.Size(116, 21);
            this.CBX_ComPorts.TabIndex = 9;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(6, 64);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(89, 13);
            this.label11.TabIndex = 8;
            this.label11.Text = "Select COM Port:";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 21.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.Location = new System.Drawing.Point(179, 14);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(174, 33);
            this.label1.TabIndex = 8;
            this.label1.Text = "UNICO Lite";
            // 
            // pictureBox1
            // 
            this.pictureBox1.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch;
            this.pictureBox1.Image = global::Unico_Lite.Properties.Resources.ST_LifeAugmented;
            this.pictureBox1.Location = new System.Drawing.Point(12, 2);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(156, 110);
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
            // 
            // LB_VersionValue
            // 
            this.LB_VersionValue.AutoSize = true;
            this.LB_VersionValue.BackColor = System.Drawing.Color.Transparent;
            this.LB_VersionValue.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LB_VersionValue.ForeColor = System.Drawing.Color.Black;
            this.LB_VersionValue.Location = new System.Drawing.Point(424, 27);
            this.LB_VersionValue.Name = "LB_VersionValue";
            this.LB_VersionValue.Size = new System.Drawing.Size(45, 16);
            this.LB_VersionValue.TabIndex = 18;
            this.LB_VersionValue.Text = "0.0.0.0";
            this.LB_VersionValue.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // LB_Version
            // 
            this.LB_Version.AutoSize = true;
            this.LB_Version.BackColor = System.Drawing.Color.Transparent;
            this.LB_Version.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.LB_Version.ForeColor = System.Drawing.Color.Black;
            this.LB_Version.Location = new System.Drawing.Point(391, 27);
            this.LB_Version.Name = "LB_Version";
            this.LB_Version.Size = new System.Drawing.Size(32, 16);
            this.LB_Version.TabIndex = 17;
            this.LB_Version.Text = "Ver.";
            this.LB_Version.TextAlign = System.Drawing.ContentAlignment.MiddleLeft;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(481, 261);
            this.Controls.Add(this.LB_VersionValue);
            this.Controls.Add(this.LB_Version);
            this.Controls.Add(this.pictureBox1);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.GB_DataOutput);
            this.Controls.Add(this.GB_DirectCommunication);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MaximizeBox = false;
            this.Name = "Form1";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Unico Lite";
            this.GB_DirectCommunication.ResumeLayout(false);
            this.GB_DirectCommunication.PerformLayout();
            this.GB_DataOutput.ResumeLayout(false);
            this.GB_DataOutput.PerformLayout();
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.pictureBox1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Button BTN_Connect;
        private System.Windows.Forms.Button BTN_Start;
        private System.Windows.Forms.ComboBox CBX_Kit;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.GroupBox GB_DirectCommunication;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox TB_Value;
        private System.Windows.Forms.TextBox TB_Address;
        private System.Windows.Forms.Button BTN_Write;
        private System.Windows.Forms.Button BTN_Read;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.GroupBox GB_DataOutput;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox TB_Val3;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox TB_Val2;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox TB_Val1;
        private System.Windows.Forms.RadioButton RB_Acc;
        private System.Windows.Forms.RadioButton RB_Gyro;
        private System.Windows.Forms.RadioButton RB_Prs;
        private System.Windows.Forms.RadioButton RB_Mag;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.ComboBox CBX_ComPorts;
        private System.Windows.Forms.Button BTN_ComRefresh;
        private System.Windows.Forms.Label LB_VersionValue;
        private System.Windows.Forms.Label LB_Version;
    }
}

