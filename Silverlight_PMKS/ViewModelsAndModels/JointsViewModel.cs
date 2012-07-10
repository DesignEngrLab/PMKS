using System;
using System.Collections.ObjectModel;
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
    public class JointsViewModel : ViewModelBase
    {
        ObservableCollection<JointData> infos;
        ICommand _command;

        public JointsViewModel()
        {
            JointsInfo = new ObservableCollection<JointData>()
            {
                new JointData {JointType = "R (pin joint)", XPos = "0.0", YPos = "0.0", LinkNames="ground, "},
                new JointData(),new JointData(),new JointData(),new JointData(),new JointData()
            };
        }

        public ObservableCollection<JointData> JointsInfo
        {
            get
            {
                return infos;
            }
            set
            {
                infos = value;
                OnPropertyChanged("JointsInfo");
            }
        }
        #region Remove Command
        public ICommand RemoveCommand
        {
            get
            {
                if (_command == null)
                {
                    _command = new DelegateCommand(RemoveCommandCanExecute, RemoveCommandExecute);
                }
                return _command;
            }
        }

        private void RemoveCommandExecute(object parameter)
        {
            int index = JointsInfo.IndexOf(parameter as JointData);
            if (index > -1 && index < JointsInfo.Count)
            {
                JointsInfo.RemoveAt(index);
            }
        }

        private bool RemoveCommandCanExecute(object parameter)
        {
            return true;
        }
        #endregion

        #region Add Command
        public ICommand AddCommand
        {
            get
            {
                if (_command == null)
                {
                    _command = new DelegateCommand(AddCommandCanExecute, AddCommandExecute);
                }
                return _command;
            }
        }

        private void AddCommandExecute(object parameter)
        {
                JointsInfo.Add(new JointData());
        }

        private bool AddCommandCanExecute(object parameter)
        {
            return true;
        }
        #endregion


        #region Open Command
        public ICommand OpenCommand
        {
            get
            {
                if (_command == null)
                {
                    _command = new DelegateCommand(OpenCommandCanExecute, OpenCommandExecute);
                }
                return _command;
            }
        }

        private void OpenCommandExecute(object parameter)
        {
            throw new NotImplementedException();
        }

        private bool OpenCommandCanExecute(object parameter)
        {
            return true;
        }
        #endregion
        #region Save Command
        public ICommand SaveCommand
        {
            get
            {
                if (_command == null)
                {
                    _command = new DelegateCommand(SaveCommandCanExecute, SaveCommandExecute);
                }
                return _command;
            }
        }

        private void SaveCommandExecute(object parameter)
        {
            throw new NotImplementedException();
        }

        private bool SaveCommandCanExecute(object parameter)
        {
            return true;
        }
        #endregion
        #region Clear Command
        public ICommand ClearCommand
        {
            get
            {
                if (_command == null)
                {
                    _command = new DelegateCommand(ClearCommandCanExecute, ClearCommandExecute);
                }
                return _command;
            }
        }

        private void ClearCommandExecute(object parameter)
        {
            throw new NotImplementedException();
        }

        private bool ClearCommandCanExecute(object parameter)
        {
            return true;
        }
        #endregion
    }

}

