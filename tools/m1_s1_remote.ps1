# Dot-sourced by the S1 deploy/collect scripts. Runs a local POSIX script on
# the board as `ssh BOARD sh -s -- ARGS` with the script on stdin, so no shell
# syntax ever appears on the ssh command line. Arguments must be plain words
# (callers validate them). Returns the remote exit status.
function Invoke-M1S1RemoteScript([string]$Board,[string]$Script,[string[]]$ScriptArgs){
  foreach($a in @($Board)+$ScriptArgs){
    if($a -notmatch '^[A-Za-z0-9@._:/+-]+$'){throw "Unsafe remote argument: $a"}
  }
  $lf=[IO.Path]::GetTempFileName()
  try {
    $text=[IO.File]::ReadAllText($Script).Replace("`r`n","`n")
    [IO.File]::WriteAllText($lf,$text,[Text.UTF8Encoding]::new($false))
    $p=Start-Process -FilePath ssh -ArgumentList (@($Board,'sh','-s','--')+$ScriptArgs) -RedirectStandardInput $lf -NoNewWindow -Wait -PassThru
    return $p.ExitCode
  } finally { Remove-Item -LiteralPath $lf -Force -ErrorAction SilentlyContinue }
}
