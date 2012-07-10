using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;

namespace PMKS_Silverlight_App
{
    public class DelegateCommand : ICommand
    {

        Predicate<object> canExecute;
        Action<object> execute;

        public DelegateCommand(Predicate<object> _canexecute, Action<object> _execute)
            : this()
        {
            canExecute = _canexecute;
            execute = _execute;
        }

        public DelegateCommand()
        {

        }

        public bool CanExecute(object parameter)
        {
            return canExecute == null ? true : canExecute(parameter);
        }

        public event EventHandler CanExecuteChanged;

        public void Execute(object parameter)
        {
            execute(parameter);
        }
    }


}
