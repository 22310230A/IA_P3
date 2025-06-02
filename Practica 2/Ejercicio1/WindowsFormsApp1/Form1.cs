using System;
using System.Diagnostics;
using System.Linq;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void btnOrdenar_Click(object sender, EventArgs e)
        {
            try
            {
                // Limpiar el resultado anterior
                flpResult.Controls.Clear();

                // Obtener y convertir los números
                int[] numeros = txtInput.Text
                    .Split(',')
                    .Select(n => int.Parse(n.Trim()))
                    .ToArray();

                // Iniciar cronómetro
                Stopwatch cronometro = new Stopwatch();
                cronometro.Start();

                // Ordenar con método de inserción
                for (int i = 1; i < numeros.Length; i++)
                {
                    int key = numeros[i];
                    int j = i - 1;

                    while (j >= 0 && numeros[j] > key)
                    {
                        numeros[j + 1] = numeros[j];
                        j--;
                    }
                    numeros[j + 1] = key;
                }

                // Detener cronómetro
                cronometro.Stop();

                // Mostrar los números ordenados en el FlowLayoutPanel
                foreach (int num in numeros)
                {
                    Label lbl = new Label();
                    lbl.Text = num.ToString();
                    lbl.AutoSize = true;
                    lbl.Margin = new Padding(5);
                    flpResult.Controls.Add(lbl);
                }

                // Mostrar tiempo transcurrido
                MessageBox.Show($"Tiempo de ordenamiento: {cronometro.Elapsed.TotalMilliseconds} ms", "Ordenamiento completo");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error: Asegúrate de ingresar solo números separados por comas.\n\n" + ex.Message);
            }
        }
    }
}
