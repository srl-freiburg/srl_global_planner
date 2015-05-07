#!/usr/bin/perl -w
use strict;
 
my @rmdirs  = ('.svn');
my @rmfiles = ();
 
sub filelist
{
    my ($dir, $regex) = @_;
    opendir(DIR, $dir) || return ();
    my @files = grep { (/$regex/) && 
			   ($_ ne '.') && ($_ ne '..') } readdir(DIR);
    closedir DIR;
    @files; 
}
 
sub dirlist 
{
    my ($dir) = @_;
    opendir(DIR, $dir) || return ();
    my @dirs = grep { (-d $dir.'/'.$_) && 
			  ($_ ne '.') && ($_ ne '..') } readdir(DIR);
    closedir DIR;
    @dirs;
}
 
sub run 
{
    my ($path) = @_;
    $path .= '/' unless $path =~ m!\/$!;
    my @dirs = dirlist($path);
 
    foreach my $dir(@dirs)
    {
	
	my $removed = 0;
	foreach my $rmdir(@rmdirs) {
	    if ($dir eq $rmdir) {
		print "Removing directory $path$dir\n";
		`rm -rf \"$path$dir\"`;
		$removed = 1; last; }}
	run($path.$dir) unless $removed;
    }
	
    my @files = filelist($path, '.*');
	
    foreach my $file(@files)
    {
	foreach my $rmfile(@rmfiles) {
	    if ($file eq $rmfile) {
		print "Removing file $path$file\n";
		`rm -rf \"$path$file\"`; last; }}
    }
}
 
if (($ARGV[0] || '') eq '--help') {
    print "Clean current directory. If parameter is given, ",
    "cleans that path instead\n";
} else {
    run($ARGV[0] || '.');
}
