namespace WindowsFormsApp2
{
    partial class Form2
    {
        private System.ComponentModel.IContainer components = null;

        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        private void InitializeComponent()
        {
            this.txtInput = new System.Windows.Forms.TextBox();
            this.btnOrdenar = new System.Windows.Forms.Button();
            this.flpResult = new System.Windows.Forms.FlowLayoutPanel();
            this.SuspendLayout();
            // 
            // txtInput
            // 
            this.txtInput.Location = new System.Drawing.Point(276, 130);
            this.txtInput.Name = "txtInput";
            this.txtInput.Size = new System.Drawing.Size(250, 26);
            this.txtInput.TabIndex = 0;
            // 
            // btnOrdenar
            // 
            this.btnOrdenar.Location = new System.Drawing.Point(550, 130);
            this.btnOrdenar.Name = "btnOrdenar";
            this.btnOrdenar.Size = new System.Drawing.Size(75, 30);
            this.btnOrdenar.TabIndex = 1;
            this.btnOrdenar.Text = "Ordenar";
            this.btnOrdenar.UseVisualStyleBackColor = true;
            this.btnOrdenar.Click += new System.EventHandler(this.btnOrdenar_Click);
            // 
            // flpResult
            // 
            this.flpResult.Location = new System.Drawing.Point(276, 180);
            this.flpResult.Name = "flpResult";
            this.flpResult.Size = new System.Drawing.Size(350, 150);
            this.flpResult.TabIndex = 2;
            // 
            // Form1
            // 
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Controls.Add(this.flpResult);
            this.Controls.Add(this.btnOrdenar);
            this.Controls.Add(this.txtInput);
            this.Name = "Form1";
            this.Text = "Ejercicio 2 - Ordenar con Tiempo";
            this.ResumeLayout(false);
            this.PerformLayout();
        }

        #endregion

        private System.Windows.Forms.TextBox txtInput;
        private System.Windows.Forms.Button btnOrdenar;
        private System.Windows.Forms.FlowLayoutPanel flpResult;
    }
}
