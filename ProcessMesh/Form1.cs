using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using System.IO;

namespace ProcessMesh
{
	/// <summary>
	/// Summary description for Form1.
	/// </summary>
	public class Form1 : System.Windows.Forms.Form
	{
		private System.Windows.Forms.Button button2;
		private System.Windows.Forms.Label label1;
		private System.Windows.Forms.Label label2;
		private System.Windows.Forms.Label label3;
		private System.Windows.Forms.TextBox txtMass;
		private System.Windows.Forms.Label txtFilename;
		private System.Windows.Forms.Button btnWrite;
		private System.Windows.Forms.TextBox txtBounciness;
		private System.Windows.Forms.TextBox txtInertia;
		private System.Windows.Forms.TextBox txtOutfilename;
		private System.Windows.Forms.Label label4;
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
			this.txtInertia = new System.Windows.Forms.TextBox();
			this.txtMass = new System.Windows.Forms.TextBox();
			this.txtBounciness = new System.Windows.Forms.TextBox();
			this.btnWrite = new System.Windows.Forms.Button();
			this.button2 = new System.Windows.Forms.Button();
			this.label1 = new System.Windows.Forms.Label();
			this.label2 = new System.Windows.Forms.Label();
			this.label3 = new System.Windows.Forms.Label();
			this.txtFilename = new System.Windows.Forms.Label();
			this.txtOutfilename = new System.Windows.Forms.TextBox();
			this.label4 = new System.Windows.Forms.Label();
			this.SuspendLayout();
			// 
			// txtInertia
			// 
			this.txtInertia.Enabled = false;
			this.txtInertia.Location = new System.Drawing.Point(16, 160);
			this.txtInertia.Name = "txtInertia";
			this.txtInertia.TabIndex = 3;
			this.txtInertia.Text = "";
			this.txtInertia.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtMass
			// 
			this.txtMass.Enabled = false;
			this.txtMass.Location = new System.Drawing.Point(16, 64);
			this.txtMass.Name = "txtMass";
			this.txtMass.TabIndex = 1;
			this.txtMass.Text = "";
			this.txtMass.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// txtBounciness
			// 
			this.txtBounciness.Enabled = false;
			this.txtBounciness.Location = new System.Drawing.Point(16, 112);
			this.txtBounciness.Name = "txtBounciness";
			this.txtBounciness.TabIndex = 2;
			this.txtBounciness.Text = "";
			this.txtBounciness.TextChanged += new System.EventHandler(this.txt_TextChanged);
			// 
			// btnWrite
			// 
			this.btnWrite.Enabled = false;
			this.btnWrite.Location = new System.Drawing.Point(184, 232);
			this.btnWrite.Name = "btnWrite";
			this.btnWrite.Size = new System.Drawing.Size(88, 23);
			this.btnWrite.TabIndex = 5;
			this.btnWrite.Text = "Write .amf file";
			this.btnWrite.Click += new System.EventHandler(this.button1_Click);
			// 
			// button2
			// 
			this.button2.Location = new System.Drawing.Point(16, 16);
			this.button2.Name = "button2";
			this.button2.TabIndex = 0;
			this.button2.Text = "Load Mesh";
			this.button2.Click += new System.EventHandler(this.button2_Click);
			// 
			// label1
			// 
			this.label1.AutoSize = true;
			this.label1.Location = new System.Drawing.Point(16, 48);
			this.label1.Name = "label1";
			this.label1.Size = new System.Drawing.Size(31, 13);
			this.label1.TabIndex = 7;
			this.label1.Text = "Mass";
			// 
			// label2
			// 
			this.label2.AutoSize = true;
			this.label2.Location = new System.Drawing.Point(16, 96);
			this.label2.Name = "label2";
			this.label2.Size = new System.Drawing.Size(63, 13);
			this.label2.TabIndex = 8;
			this.label2.Text = "Bounciness";
			// 
			// label3
			// 
			this.label3.AutoSize = true;
			this.label3.Location = new System.Drawing.Point(16, 144);
			this.label3.Name = "label3";
			this.label3.Size = new System.Drawing.Size(90, 13);
			this.label3.TabIndex = 9;
			this.label3.Text = "Rotational Inertia";
			// 
			// txtFilename
			// 
			this.txtFilename.Location = new System.Drawing.Point(136, 16);
			this.txtFilename.Name = "txtFilename";
			this.txtFilename.Size = new System.Drawing.Size(144, 200);
			this.txtFilename.TabIndex = 6;
			// 
			// txtOutfilename
			// 
			this.txtOutfilename.Location = new System.Drawing.Point(16, 232);
			this.txtOutfilename.Name = "txtOutfilename";
			this.txtOutfilename.Size = new System.Drawing.Size(152, 20);
			this.txtOutfilename.TabIndex = 4;
			this.txtOutfilename.Text = "";
			// 
			// label4
			// 
			this.label4.AutoSize = true;
			this.label4.Location = new System.Drawing.Point(16, 216);
			this.label4.Name = "label4";
			this.label4.Size = new System.Drawing.Size(56, 13);
			this.label4.TabIndex = 10;
			this.label4.Text = "Output file";
			// 
			// Form1
			// 
			this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
			this.ClientSize = new System.Drawing.Size(292, 271);
			this.Controls.AddRange(new System.Windows.Forms.Control[] {
																		  this.label4,
																		  this.txtOutfilename,
																		  this.txtFilename,
																		  this.label3,
																		  this.label2,
																		  this.label1,
																		  this.button2,
																		  this.btnWrite,
																		  this.txtBounciness,
																		  this.txtMass,
																		  this.txtInertia});
			this.Name = "Form1";
			this.Text = "Mesh Processor";
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

		private void button2_Click(object sender, System.EventArgs e)
		{
			OpenFileDialog dlg = new OpenFileDialog();
			dlg.Filter = "X File(.x)|*.x";
			if( dlg.ShowDialog() == DialogResult.OK )
			{
				txtFilename.Text = dlg.FileName;
				txtMass.Enabled = true;
				txtBounciness.Enabled = true;
				txtInertia.Enabled = true;
				txtOutfilename.Text = dlg.FileName.Substring(0,dlg.FileName.LastIndexOf('.')) + ".amf";
			}
		}

		private void button1_Click(object sender, System.EventArgs e)
		{
			try
			{
				float mass = Convert.ToSingle( txtMass.Text );
				float bounciness = Convert.ToSingle( txtBounciness.Text );
				float inertia = Convert.ToSingle( txtInertia.Text );
				System.Diagnostics.Process p = new System.Diagnostics.Process();
				p.StartInfo.RedirectStandardError = true;
				p.StartInfo.FileName = Application.StartupPath + "\\ProcessMesh.exe";
				p.StartInfo.UseShellExecute = false;
				p.StartInfo.Arguments = '\"' + txtFilename.Text + "\" \"" + txtOutfilename.Text + "\" " + Convert.ToString(mass) + ' ' + Convert.ToString(bounciness) + ' ' + Convert.ToString(inertia);
				p.Start();
				string output = p.StandardError.ReadToEnd();
				p.WaitForExit();

				if( output != "" )
					MessageBox.Show( output, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error );
				else
					MessageBox.Show( "Done!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information );
			}
			catch( Exception err )
			{
				MessageBox.Show( err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error );
			}
		}

		private void txt_TextChanged(object sender, System.EventArgs e)
		{
			try
			{
				Convert.ToSingle( txtMass.Text );
				Convert.ToSingle( txtBounciness.Text );
				Convert.ToSingle( txtInertia.Text );
				btnWrite.Enabled = true;
			}
			catch
			{
				btnWrite.Enabled = false;
            }				
		}
	}
}
