using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using System.IO;

namespace SceneMaker
{
	/// <summary>
	/// Summary description for Form1.
	/// </summary>
	public class Form1 : System.Windows.Forms.Form
	{
		private System.Windows.Forms.Button btnAddObject;
		private System.Windows.Forms.Label label1;
		private System.Windows.Forms.Label label2;
		private System.Windows.Forms.Button btnWriteScene;
		private System.Windows.Forms.Label label4;
		private System.Windows.Forms.TextBox txtOutfilename;
		private System.Windows.Forms.Label label5;
		private System.Windows.Forms.TextBox txtPosX;
		private System.Windows.Forms.TextBox txtRotX;
		private System.Windows.Forms.TextBox txtPosY;
		private System.Windows.Forms.TextBox txtPosZ;
		private System.Windows.Forms.TextBox txtRotY;
		private System.Windows.Forms.TextBox txtRotZ;
		private System.Windows.Forms.TextBox txtRotAngle;
		private System.Windows.Forms.ListBox lstObjects;
		private System.Windows.Forms.Button btnLoadObject;
		private System.Windows.Forms.Label txtCurObject;
		private System.Windows.Forms.Label label3;
		private System.Windows.Forms.CheckBox chkMobile;
		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

		public Form1()
		{
			//
			// Required for Windows Form Designer support
			//
			InitializeComponent();

			//
			// TODO: Add any constructor code after InitializeComponent call
			//
		}

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		protected override void Dispose( bool disposing )
		{
			if( disposing )
			{
				if (components != null) 
				{
					components.Dispose();
				}
			}
			base.Dispose( disposing );
		}

		#region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			this.btnAddObject = new System.Windows.Forms.Button();
			this.txtPosX = new System.Windows.Forms.TextBox();
			this.txtRotX = new System.Windows.Forms.TextBox();
			this.label1 = new System.Windows.Forms.Label();
			this.label2 = new System.Windows.Forms.Label();
			this.txtPosY = new System.Windows.Forms.TextBox();
			this.txtPosZ = new System.Windows.Forms.TextBox();
			this.txtRotY = new System.Windows.Forms.TextBox();
			this.txtRotZ = new System.Windows.Forms.TextBox();
			this.txtRotAngle = new System.Windows.Forms.TextBox();
			this.btnWriteScene = new System.Windows.Forms.Button();
			this.label4 = new System.Windows.Forms.Label();
			this.txtOutfilename = new System.Windows.Forms.TextBox();
			this.label5 = new System.Windows.Forms.Label();
			this.lstObjects = new System.Windows.Forms.ListBox();
			this.btnLoadObject = new System.Windows.Forms.Button();
			this.txtCurObject = new System.Windows.Forms.Label();
			this.label3 = new System.Windows.Forms.Label();
			this.chkMobile = new System.Windows.Forms.CheckBox();
			this.SuspendLayout();
			// 
			// btnAddObject
			// 
			this.btnAddObject.Enabled = false;
			this.btnAddObject.Location = new System.Drawing.Point(120, 184);
			this.btnAddObject.Name = "btnAddObject";
			this.btnAddObject.Size = new System.Drawing.Size(56, 23);
			this.btnAddObject.TabIndex = 9;
			this.btnAddObject.Text = "Add ->";
			this.btnAddObject.Click += new System.EventHandler(this.btnAddObject_Click);
			// 
			// txtPosX
			// 
			this.txtPosX.Enabled = false;
			this.txtPosX.Location = new System.Drawing.Point(16, 64);
			this.txtPosX.Name = "txtPosX";
			this.txtPosX.Size = new System.Drawing.Size(48, 20);
			this.txtPosX.TabIndex = 1;
			this.txtPosX.Text = "";
			this.txtPosX.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtRotX
			// 
			this.txtRotX.Enabled = false;
			this.txtRotX.Location = new System.Drawing.Point(16, 112);
			this.txtRotX.Name = "txtRotX";
			this.txtRotX.Size = new System.Drawing.Size(48, 20);
			this.txtRotX.TabIndex = 4;
			this.txtRotX.Text = "";
			this.txtRotX.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// label1
			// 
			this.label1.AutoSize = true;
			this.label1.Location = new System.Drawing.Point(16, 48);
			this.label1.Name = "label1";
			this.label1.Size = new System.Drawing.Size(45, 13);
			this.label1.TabIndex = 12;
			this.label1.Text = "Position";
			// 
			// label2
			// 
			this.label2.AutoSize = true;
			this.label2.Location = new System.Drawing.Point(16, 96);
			this.label2.Name = "label2";
			this.label2.Size = new System.Drawing.Size(87, 13);
			this.label2.TabIndex = 13;
			this.label2.Text = "Orientation: Axis";
			// 
			// txtPosY
			// 
			this.txtPosY.Enabled = false;
			this.txtPosY.Location = new System.Drawing.Point(72, 64);
			this.txtPosY.Name = "txtPosY";
			this.txtPosY.Size = new System.Drawing.Size(48, 20);
			this.txtPosY.TabIndex = 2;
			this.txtPosY.Text = "";
			this.txtPosY.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtPosZ
			// 
			this.txtPosZ.Enabled = false;
			this.txtPosZ.Location = new System.Drawing.Point(128, 64);
			this.txtPosZ.Name = "txtPosZ";
			this.txtPosZ.Size = new System.Drawing.Size(48, 20);
			this.txtPosZ.TabIndex = 3;
			this.txtPosZ.Text = "";
			this.txtPosZ.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtRotY
			// 
			this.txtRotY.Enabled = false;
			this.txtRotY.Location = new System.Drawing.Point(72, 112);
			this.txtRotY.Name = "txtRotY";
			this.txtRotY.Size = new System.Drawing.Size(48, 20);
			this.txtRotY.TabIndex = 5;
			this.txtRotY.Text = "";
			this.txtRotY.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtRotZ
			// 
			this.txtRotZ.Enabled = false;
			this.txtRotZ.Location = new System.Drawing.Point(128, 112);
			this.txtRotZ.Name = "txtRotZ";
			this.txtRotZ.Size = new System.Drawing.Size(48, 20);
			this.txtRotZ.TabIndex = 6;
			this.txtRotZ.Text = "";
			this.txtRotZ.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtRotAngle
			// 
			this.txtRotAngle.Enabled = false;
			this.txtRotAngle.Location = new System.Drawing.Point(128, 152);
			this.txtRotAngle.Name = "txtRotAngle";
			this.txtRotAngle.Size = new System.Drawing.Size(48, 20);
			this.txtRotAngle.TabIndex = 7;
			this.txtRotAngle.Text = "";
			this.txtRotAngle.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// btnWriteScene
			// 
			this.btnWriteScene.Enabled = false;
			this.btnWriteScene.Location = new System.Drawing.Point(184, 232);
			this.btnWriteScene.Name = "btnWriteScene";
			this.btnWriteScene.Size = new System.Drawing.Size(88, 23);
			this.btnWriteScene.TabIndex = 11;
			this.btnWriteScene.Text = "Write Scene";
			this.btnWriteScene.Click += new System.EventHandler(this.btnWriteScene_Click);
			// 
			// label4
			// 
			this.label4.AutoSize = true;
			this.label4.Location = new System.Drawing.Point(16, 216);
			this.label4.Name = "label4";
			this.label4.Size = new System.Drawing.Size(56, 13);
			this.label4.TabIndex = 17;
			this.label4.Text = "Output file";
			// 
			// txtOutfilename
			// 
			this.txtOutfilename.Location = new System.Drawing.Point(16, 232);
			this.txtOutfilename.Name = "txtOutfilename";
			this.txtOutfilename.Size = new System.Drawing.Size(152, 20);
			this.txtOutfilename.TabIndex = 10;
			this.txtOutfilename.Text = "";
			this.txtOutfilename.TextChanged += new System.EventHandler(this.txtOutfilename_TextChanged);
			// 
			// label5
			// 
			this.label5.AutoSize = true;
			this.label5.Location = new System.Drawing.Point(143, 136);
			this.label5.Name = "label5";
			this.label5.Size = new System.Drawing.Size(33, 13);
			this.label5.TabIndex = 14;
			this.label5.Text = "Angle";
			// 
			// lstObjects
			// 
			this.lstObjects.Location = new System.Drawing.Point(184, 16);
			this.lstObjects.Name = "lstObjects";
			this.lstObjects.Size = new System.Drawing.Size(176, 199);
			this.lstObjects.TabIndex = 18;
			this.lstObjects.TabStop = false;
			// 
			// btnLoadObject
			// 
			this.btnLoadObject.Location = new System.Drawing.Point(16, 16);
			this.btnLoadObject.Name = "btnLoadObject";
			this.btnLoadObject.TabIndex = 0;
			this.btnLoadObject.Text = "Load Object";
			this.btnLoadObject.Click += new System.EventHandler(this.btnLoadObject_Click);
			// 
			// txtCurObject
			// 
			this.txtCurObject.Location = new System.Drawing.Point(32, 192);
			this.txtCurObject.Name = "txtCurObject";
			this.txtCurObject.Size = new System.Drawing.Size(80, 16);
			this.txtCurObject.TabIndex = 16;
			// 
			// label3
			// 
			this.label3.AutoSize = true;
			this.label3.Location = new System.Drawing.Point(16, 176);
			this.label3.Name = "label3";
			this.label3.Size = new System.Drawing.Size(81, 13);
			this.label3.TabIndex = 15;
			this.label3.Text = "Current Object:";
			// 
			// chkMobile
			// 
			this.chkMobile.Enabled = false;
			this.chkMobile.Location = new System.Drawing.Point(24, 144);
			this.chkMobile.Name = "chkMobile";
			this.chkMobile.Size = new System.Drawing.Size(64, 24);
			this.chkMobile.TabIndex = 8;
			this.chkMobile.Text = "mobile";
			// 
			// Form1
			// 
			this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
			this.ClientSize = new System.Drawing.Size(376, 271);
			this.Controls.AddRange(new System.Windows.Forms.Control[] {
																		  this.chkMobile,
																		  this.label3,
																		  this.txtCurObject,
																		  this.btnLoadObject,
																		  this.lstObjects,
																		  this.label5,
																		  this.label4,
																		  this.txtOutfilename,
																		  this.btnWriteScene,
																		  this.txtRotAngle,
																		  this.txtRotZ,
																		  this.txtRotY,
																		  this.txtPosZ,
																		  this.txtPosY,
																		  this.label2,
																		  this.label1,
																		  this.txtRotX,
																		  this.txtPosX,
																		  this.btnAddObject});
			this.Name = "Form1";
			this.Text = "SceneMaker";
			this.ResumeLayout(false);

		}
		#endregion

		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		static void Main() 
		{
			Application.Run(new Form1());
		}

		private void btnLoadObject_Click(object sender, System.EventArgs e)
		{
			OpenFileDialog dlg = new OpenFileDialog();
			dlg.Filter = "AJG Mesh File(.amf)|*.amf";
			if( dlg.ShowDialog() == DialogResult.OK )
			{
				txtCurObject.Text = dlg.FileName.Substring(dlg.FileName.LastIndexOf('\\') + 1);
				txtPosX.Enabled = true;
				txtPosY.Enabled = true;
				txtPosZ.Enabled = true;
				txtRotX.Enabled = true;
				txtRotY.Enabled = true;
				txtRotZ.Enabled = true;
				txtRotAngle.Enabled = true;
				chkMobile.Enabled = true;
				chkMobile.Checked = true;
			}
		}

		private void txt_TextChanged(object sender, System.EventArgs e)
		{
			try
			{
				Convert.ToSingle( txtPosX.Text );
				Convert.ToSingle( txtPosY.Text );
				Convert.ToSingle( txtPosZ.Text );
				Convert.ToSingle( txtRotX.Text );
				Convert.ToSingle( txtRotY.Text );
				Convert.ToSingle( txtRotZ.Text );
				Convert.ToSingle( txtRotAngle.Text );
				btnAddObject.Enabled = true;
			}
			catch
			{
				btnAddObject.Enabled = false;
			}				
		}

		private void btnAddObject_Click(object sender, System.EventArgs e)
		{
			lstObjects.Items.Add( txtCurObject.Text + ' ' + txtPosX.Text + ' ' + txtPosY.Text + ' ' + txtPosZ.Text
				+ ' ' + txtRotX.Text + ' ' + txtRotY.Text + ' ' + txtRotZ.Text + ' ' + txtRotAngle.Text + ' ' + (chkMobile.Checked?'1':'0') );
			txtCurObject.Text = "";
			txtPosX.Text = "";
			txtPosY.Text = "";
			txtPosZ.Text = "";
			txtRotX.Text = "";
			txtRotY.Text = "";
			txtRotZ.Text = "";
			txtRotAngle.Text = "";
			txtPosX.Enabled = false;
			txtPosY.Enabled = false;
			txtPosZ.Enabled = false;
			txtRotX.Enabled = false;
			txtRotY.Enabled = false;
			txtRotZ.Enabled = false;
			txtRotAngle.Enabled = false;
			btnAddObject.Enabled = false;
			chkMobile.Enabled = false;
			chkMobile.Checked = false;
		}

		private void txtOutfilename_TextChanged(object sender, System.EventArgs e)
		{
			if( txtOutfilename.Text != "" && lstObjects.Items.Count != 0 )
			{
				btnWriteScene.Enabled = true;
			}
			else
			{
				btnWriteScene.Enabled = false;
			}
		}

		private void btnWriteScene_Click(object sender, System.EventArgs e)
		{
			try
			{
				StreamWriter SW;
				SW = File.CreateText( txtOutfilename.Text );
				SW.WriteLine( "##Physics Simulator Scene Description File##" );
				SW.WriteLine( "Camera 0 0 30 45 15" );
				for( int i = 0; i < lstObjects.Items.Count; i++ )
				{
					SW.WriteLine( lstObjects.Items[i].ToString() );
				}
				SW.Close();
				MessageBox.Show( "Done!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information );
			}
			catch( Exception err )
			{
				MessageBox.Show( err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error );
			}

			txtOutfilename.Text = "";
			txtCurObject.Text = "";
			txtPosX.Text = "";
			txtPosY.Text = "";
			txtPosZ.Text = "";
			txtRotX.Text = "";
			txtRotY.Text = "";
			txtRotZ.Text = "";
			txtRotAngle.Text = "";
			txtPosX.Enabled = false;
			txtPosY.Enabled = false;
			txtPosZ.Enabled = false;
			txtRotX.Enabled = false;
			txtRotY.Enabled = false;
			txtRotZ.Enabled = false;
			txtRotAngle.Enabled = false;
			btnAddObject.Enabled = false;
			chkMobile.Enabled = false;
			chkMobile.Checked = false;
			lstObjects.Items.Clear();
		}
	}
}
