#!/bin/sh
cd ../helisharp/HeliSharpLib
dotnet publish
cd -
cp -f ../helisharp/HeliSharpLib/bin/Debug/netstandard2.0/publish/*.dll Assets/UnityHeliKit/Plugins/

