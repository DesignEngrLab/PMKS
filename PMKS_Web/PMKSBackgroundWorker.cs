
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;

namespace SL_BackgroundWorker_CS
{
    public  class PMKSBackgroundWorker : BackgroundWorker
    {
        private PlanarMechanismSimulator.Simulator pmks;

        internal static void ParseData()
        {
        }

        public PMKSBackgroundWorker()
        {
            WorkerReportsProgress = true;
            WorkerSupportsCancellation = true;
            DoWork += new DoWorkEventHandler(bw_DoWork);
            ProgressChanged += new ProgressChangedEventHandler(bw_ProgressChanged);
            RunWorkerCompleted += new RunWorkerCompletedEventHandler(bw_RunWorkerCompleted);
        }
        private void buttonStart_Click(object sender, RoutedEventArgs e)
        {
            if (IsBusy != true)
            {
                RunWorkerAsync();
            }
        }
        private void buttonCancel_Click(object sender, RoutedEventArgs e)
        {
            if (WorkerSupportsCancellation == true)
            {
                CancelAsync();
            }
        }
        private void bw_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;

            for (int i = 1; (i <= 10); i++)
            {
                if ((worker.CancellationPending == true))
                {
                    e.Cancel = true;
                    break;
                }
                else
                {
                    // Perform a time consuming operation and report progress.
                    System.Threading.Thread.Sleep(500);
                    worker.ReportProgress((i * 10));
                }
            }
        }
        private void bw_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            if ((e.Cancelled == true))
            {
              //  this.tbProgress.Text = "Canceled!";
            }

            else if (!(e.Error == null))
            {
              //  this.tbProgress.Text = ("Error: " + e.Error.Message);
            }

            else
            {
             //   this.tbProgress.Text = "Done!";
            }
        }
        private void bw_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
           // this.tbProgress.Text = (e.ProgressPercentage.ToString() + "%");
        }
    }
}