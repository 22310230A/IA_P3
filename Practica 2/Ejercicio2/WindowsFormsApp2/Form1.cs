using System;
using System.Diagnostics;
using System.Linq;
using System.Windows.Forms;

namespace WindowsFormsApp2
{
    public partial class Form2 : Form
    {
        public Form2()
        {
            InitializeComponent();
        }

        private void btnOrdenar_Click(object sender, EventArgs e)
        {
            flpResult.Controls.Clear(); // Limpiar resultados anteriores

            string input = txtInput.Text;
            if (string.IsNullOrWhiteSpace(input))
            {
                MessageBox.Show("Por favor ingresa números separados por coma.");
                return;
            }

            try
            {
                int[] numeros = input
                    .Split(new char[] { ',' }, StringSplitOptions.RemoveEmptyEntries)
                    .Select(n => int.Parse(n.Trim()))
                    .ToArray();

                Stopwatch sw = Stopwatch.StartNew();

                Array.Sort(numeros);

                sw.Stop();

                // Mostrar números ordenados en FlowLayoutPanel como Labels
                foreach (int num in numeros)
                {
                    Label lbl = new Label();
                    lbl.Text = num.ToString();
                    lbl.AutoSize = true;
                    lbl.Margin = new Padding(5);
                    flpResult.Controls.Add(lbl);
                }

                // Mostrar el tiempo transcurrido
                Label lblTiempo = new Label();
                lblTiempo.Text = $"Tiempo transcurrido: {sw.ElapsedMilliseconds} ms";
                lblTiempo.AutoSize = true;
                lblTiempo.Margin = new Padding(10);
                flpResult.Controls.Add(lblTiempo);
            }
            catch (FormatException)
            {
                MessageBox.Show("Asegúrate de ingresar sólo números separados por comas.");
            }
        }
    }
}

